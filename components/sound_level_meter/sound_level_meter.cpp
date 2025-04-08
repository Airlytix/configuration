#include "sound_level_meter.h"
#include "esphome/core/hal.h"
#include "dsps_biquad.h"

namespace esphome
{
namespace sound_level_meter
{

static const char *const TAG = "sound_level_meter";

// By definition dBFS value of a full-scale sine wave equals to 0.
// Since the RMS of the full-scale sine wave is 1/sqrt(2), multiplying rms(signal) by sqrt(2)
// ensures that the formula evaluates to 0 when signal is a full-scale sine wave.
// This is equivalent to adding DBFS_OFFSET
// see: https://dsp.stackexchange.com/a/50947/65262
static constexpr float DBFS_OFFSET = 20 * log10(sqrt(2));

struct AudioChunk {
    std::vector<float> data;
};

struct PublishState {
    SoundLevelMeterSensor *sensor;
    float data;
};


uint32_t SoundLevelMeter::get_sample_rate() { return this->sample_rate_; }

void SoundLevelMeter::dump_config() {
    ESP_LOGCONFIG(TAG, "Sound Level Meter:");
    ESP_LOGCONFIG(TAG, "  Sample Rate %u", this->get_sample_rate());
    ESP_LOGCONFIG(TAG, "  Bits Per Sample %u", this->bits_per_sample_);
    if (this->get_mic_sensitivity().has_value())
        ESP_LOGCONFIG(TAG, "  Mic Sensitivity %fdB", *this->get_mic_sensitivity());
    if (this->get_mic_sensitivity_ref().has_value())
        ESP_LOGCONFIG(TAG, "  Mic Sensitivity Ref %fdB", *this->get_mic_sensitivity_ref());
    if (this->get_offset().has_value())
        ESP_LOGCONFIG(TAG, "  Offset %fdB", *this->get_offset());
    ESP_LOGCONFIG(TAG, "  Warmup Interval: %u ms", this->warmup_interval_);
    LOG_UPDATE_INTERVAL(this);
    if (this->groups_.size() > 0) {
      ESP_LOGCONFIG(TAG, "  Groups:");
      for (int i = 0; i < this->groups_.size(); i++) {
        ESP_LOGCONFIG(TAG, "    Group %u:", i);
        this->groups_[i]->dump_config("      ");
      }
    }
  }

void SoundLevelMeter::setup()
{
    ESP_LOGCONFIG(TAG, "Setting up Sound Level Meter...");
    {
        if (this->pdm_)
        {
            if (this->parent_->get_port() != I2S_NUM_0)
            {
                ESP_LOGE(TAG, "PDM only works on I2S0!");
                this->mark_failed();
                return;
            }
        }
    }

    if (!this->parent_->try_lock()) {
        this->mark_failed();
        return;
    }
    i2s_driver_config_t config = {
        .mode = (i2s_mode_t) (this->i2s_mode_ | I2S_MODE_RX),
        .sample_rate = this->sample_rate_,
        .bits_per_sample = this->bits_per_sample_,
        .channel_format = this->channel_,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 256,
        .use_apll = this->use_apll_,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0,
        .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        .bits_per_chan = this->bits_per_channel_,
    };

    esp_err_t err;

    {
    if (this->pdm_)
        config.mode = (i2s_mode_t) (config.mode | I2S_MODE_PDM);

    err = i2s_driver_install(this->parent_->get_port(), &config, 0, nullptr);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Error installing I2S driver: %s", esp_err_to_name(err));
        this->status_set_error();
        return;
    }

    i2s_pin_config_t pin_config = this->parent_->get_pin_config();
    pin_config.data_in_num = this->din_pin_;

    err = i2s_set_pin(this->parent_->get_port(), &pin_config);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Error setting I2S pin: %s", esp_err_to_name(err));
        this->status_set_error();
        return;
    }
    }

    this->publish_queue_ = xQueueCreate(8, sizeof(PublishState));
    if (this->publish_queue_ == nullptr) {
        ESP_LOGE(TAG, "Could not create publish queue!");
        this->mark_failed();
        return;
    }

    xTaskCreatePinnedToCore(SoundLevelMeter::task, "sound_level_meter", 4095, this, 3, nullptr, 1);

    this->status_clear_error();
} 

void SoundLevelMeter::loop()
{
    PublishState ps;
    if (xQueueReceive(this->publish_queue_, &ps, 0) == pdTRUE) {
        if (ps.sensor != nullptr) {
            ps.sensor->publish_state(ps.data);
        }
    }
}

void SoundLevelMeter::task(void *param) {
    SoundLevelMeter *this_ = reinterpret_cast<SoundLevelMeter *>(param);

    std::vector<int32_t> buffer;
    buffer.resize(32 * this_->get_sample_rate() / 1000);
    std::vector<float> buffer_float;
    buffer_float.resize(buffer.size());

    this_->warmup_start_ = millis();

    while (true) {
        size_t frames_got = this_->read_(buffer.data(), buffer.size(), buffer_float);
        if (frames_got == 0) {
            taskYIELD();
            continue;
        }

        if (millis() - this_->warmup_start_ < this_->warmup_interval_) {
            taskYIELD();
            continue;
        }


        for (auto *g : this_->groups_) {
            g->process(buffer_float);
        }

        taskYIELD();
    }
}

size_t SoundLevelMeter::read_(int32_t *buf, size_t frames_requested, std::vector<float> &buffer) {
     // 'frames_requested' is how many 32-bit frames we want.
    // Convert that to bytes for i2s_read:
    const size_t bytes_to_read = frames_requested * sizeof(int32_t);
    size_t bytes_read = 0;

    // Perform an I2S read with up to a 100ms timeout
    esp_err_t err = i2s_read(
        this->parent_->get_port(),  // which I2S port
        buf,                        // destination buffer (int32_t*)
        bytes_to_read,             // how many bytes we want
        &bytes_read,               // how many bytes we actually got
        (100 / portTICK_PERIOD_MS)
    );

    // Handle errors or zero data
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Error reading from I2S microphone: %s", esp_err_to_name(err));
        this->status_set_warning();
        return 0;
    }
    if (bytes_read == 0) {
        // Timed out or no data arrived
        this->status_set_warning();
        return 0;
    }
    this->status_clear_warning();

    // Number of 32-bit frames actually read
    size_t frames_read = bytes_read / sizeof(int32_t);

    // Now parse each 32-bit frame as top-24-bit audio.
    // We'll fill our 'float_buffer_' so the processing stage can see float data.
    buffer.resize(frames_read);

    for (size_t i = 0; i < frames_read; i++) {
        // The ICS-43434 uses the top 24 bits.
        // Shift out the bottom 8 bits, sign-extend from 24 bits:
        int32_t s24 = (buf[i] << 8) >> 8;

        // Scale to the range [-1.0, +1.0].
        // Full-scale 24-bit is ±(1<<23).
        float sample_f = static_cast<float>(s24) / static_cast<float>(1 << 23);

        buffer[i] = sample_f;
    }

    // Return how many frames of float data we prepared in float_buffer_.
    return frames_read;
}

void SoundLevelMeter::reset() {
    for (auto *g : this->groups_)
      g->reset();
  }

/* SensorGroup */

void SensorGroup::set_parent(SoundLevelMeter *parent) { this->parent_ = parent; }
void SensorGroup::add_sensor(SoundLevelMeterSensor *sensor) { this->sensors_.push_back(sensor); }
void SensorGroup::add_group(SensorGroup *group) { this->groups_.push_back(group); }
void SensorGroup::add_filter(Filter *filter) { this->filters_.push_back(filter); }

void SensorGroup::dump_config(const char *prefix)
{
    ESP_LOGCONFIG(TAG, "%sSensors:", prefix);
    for (auto *s : this->sensors_)
        LOG_SENSOR((std::string(prefix) + "  ").c_str(), "Sound Pressure Level", s);

    if (this->groups_.size() > 0)
    {
        ESP_LOGCONFIG(TAG, "%sGroups:", prefix);
        for (int i = 0; i < this->groups_.size(); i++)
        {
            ESP_LOGCONFIG(TAG, "%s  Group %u:", prefix, i);
            this->groups_[i]->dump_config((std::string(prefix) + "    ").c_str());
        }
    }
}

void SensorGroup::process(std::vector<float> &buffer)
{
    std::vector<float> &&data = this->filters_.size() > 0 ? std::vector<float>(buffer) : buffer;
    if (this->filters_.size() > 0) {
        for (auto f : this->filters_) {
            f->process(data);
        }
    }

    for (auto s : this->sensors_) {
        s->process(data);
    }

    for (auto g : this->groups_) {
        g->process(data);
    }
}

void SensorGroup::reset()
{
    for (auto f : this->filters_)
        f->reset();
    for (auto s : this->sensors_)
        s->reset();
    for (auto g : this->groups_)
        g->reset();
}

/* SoundLevelMeterSensor */

void SoundLevelMeterSensor::set_parent(SoundLevelMeter *parent)
{
    this->parent_ = parent;
    this->update_samples_ = parent->get_sample_rate() * (parent->get_update_interval() / 1000.f);
}

void SoundLevelMeterSensor::set_update_interval(uint32_t update_interval)
{
    this->update_samples_ = this->parent_->get_sample_rate() * (update_interval / 1000.f);
}

void SoundLevelMeterSensor::queue_publish_state(float state)
{
    PublishState ps;
    ps.sensor = this;
    ps.data = state;
    BaseType_t ok = xQueueSend(this->parent_->publish_queue_, &ps, 0);
    if (ok != pdTRUE) {
        ESP_LOGW(TAG, "Failed to publish state!");
    }
}

float SoundLevelMeterSensor::adjust_dB(float dB, bool is_rms)
{
    // see: https://dsp.stackexchange.com/a/50947/65262
    if (is_rms)
        dB += DBFS_OFFSET;

    // see: https://invensense.tdk.com/wp-content/uploads/2015/02/AN-1112-v1.1.pdf
    // dBSPL = dBFS + mic_sensitivity_ref - mic_sensitivity
    // e.g. dBSPL = dBFS + 94 - (-26) = dBFS + 120
    if (this->parent_->get_mic_sensitivity().has_value() && this->parent_->get_mic_sensitivity_ref().has_value())
        dB += *this->parent_->get_mic_sensitivity_ref() - *this->parent_->get_mic_sensitivity();

    if (this->parent_->get_offset().has_value())
        dB += *this->parent_->get_offset();

    return dB;
}

/* SoundLevelMeterSensorEq */

void SoundLevelMeterSensorEq::process(std::vector<float> &buffer)
{
    // as adding small floating point numbers with large ones might lead
    // to precision loss, we first accumulate local sum for entire buffer
    // and only in the end add it to global sum which could become quite large
    // for large accumulating periods (like 1 hour), therefore global sum (this->sum_)
    // is of type double
    float local_sum = 0;
    for (int i = 0; i < buffer.size(); i++)
    {
        local_sum += buffer[i] * buffer[i];
        this->count_++;
        if (this->count_ == this->update_samples_)
        {
            float dB = 10 * log10((this->sum_ + local_sum) / this->count_);
            dB = this->adjust_dB(dB);
            this->queue_publish_state(dB);
            this->sum_ = 0;
            this->count_ = 0;
            local_sum = 0;
        }
    }
    this->sum_ += local_sum;
}

void SoundLevelMeterSensorEq::reset()
{
    this->sum_ = 0.;
    this->count_ = 0;
    this->queue_publish_state(NAN);
}

/* SoundLevelMeterSensorMax */

void SoundLevelMeterSensorMax::set_window_size(uint32_t window_size)
{
    this->window_samples_ = this->parent_->get_sample_rate() * (window_size / 1000.f);
}

void SoundLevelMeterSensorMax::process(std::vector<float> &buffer)
{
    for (int i = 0; i < buffer.size(); i++)
    {
        this->sum_ += buffer[i] * buffer[i];
        this->count_sum_++;
        if (this->count_sum_ == this->window_samples_)
        {
            this->max_ = std::max(this->max_, this->sum_ / this->count_sum_);
            this->sum_ = 0.f;
            this->count_sum_ = 0;
        }
        this->count_max_++;
        if (this->count_max_ == this->update_samples_)
        {
            float dB = 10 * log10(this->max_);
            dB = this->adjust_dB(dB);
            this->queue_publish_state(dB);
            this->max_ = std::numeric_limits<float>::min();
            this->count_max_ = 0;
        }
    }
}

void SoundLevelMeterSensorMax::reset()
{
    this->sum_ = 0.f;
    this->max_ = std::numeric_limits<float>::min();
    this->count_max_ = 0;
    this->count_sum_ = 0;
    this->queue_publish_state(NAN);
}

/* SoundLevelMeterSensorMin */

void SoundLevelMeterSensorMin::set_window_size(uint32_t window_size)
{
    this->window_samples_ = this->parent_->get_sample_rate() * (window_size / 1000.f);
}

void SoundLevelMeterSensorMin::process(std::vector<float> &buffer)
{
    for (int i = 0; i < buffer.size(); i++)
    {
        this->sum_ += buffer[i] * buffer[i];
        this->count_sum_++;
        if (this->count_sum_ == this->window_samples_)
        {
            this->min_ = std::min(this->min_, this->sum_ / this->count_sum_);
            this->sum_ = 0.f;
            this->count_sum_ = 0;
        }
        this->count_min_++;
        if (this->count_min_ == this->update_samples_)
        {
            float dB = 10 * log10(this->min_);
            dB = this->adjust_dB(dB);
            this->queue_publish_state(dB);
            this->min_ = std::numeric_limits<float>::max();
            this->count_min_ = 0;
        }
    }
}

void SoundLevelMeterSensorMin::reset()
{
    this->sum_ = 0.f;
    this->min_ = std::numeric_limits<float>::max();
    this->count_min_ = 0;
    this->count_sum_ = 0;
    this->queue_publish_state(NAN);
}

/* SoundLevelMeterSensorPeak */

void SoundLevelMeterSensorPeak::process(std::vector<float> &buffer)
{
    for (int i = 0; i < buffer.size(); i++)
    {
        this->peak_ = std::max(this->peak_, abs(buffer[i]));
        this->count_++;
        if (this->count_ == this->update_samples_)
        {
            float dB = 20 * log10(this->peak_);
            dB = this->adjust_dB(dB, false);
            this->queue_publish_state(dB);
            this->peak_ = 0.f;
            this->count_ = 0;
        }
    }
}

void SoundLevelMeterSensorPeak::reset()
{
    this->peak_ = 0.f;
    this->count_ = 0;
    this->queue_publish_state(NAN);
}

/* SOS_Filter */

SOS_Filter::SOS_Filter(std::initializer_list<std::initializer_list<float>> &&coeffs)
{
    this->coeffs_.resize(coeffs.size());
    this->state_.resize(coeffs.size(), {});
    int i = 0;
    for (auto &row : coeffs)
        std::copy(row.begin(), row.end(), coeffs_[i++].begin());
}

// direct form 2 transposed
void SOS_Filter::process(std::vector<float> &data)
{
    int n = data.size();
    int m = this->coeffs_.size();
    for (int j = 0; j < m; j++)
    {
        for (int i = 0; i < n; i++)
        {
            // y[i] = b0 * x[i] + s0
            float yi = this->coeffs_[j][0] * data[i] + this->state_[j][0];
            // s0 = b1 * x[i] - a1 * y[i] + s1
            this->state_[j][0] = this->coeffs_[j][1] * data[i] - this->coeffs_[j][3] * yi + this->state_[j][1];
            // s1 = b2 * x[i] - a2 * y[i]
            this->state_[j][1] = this->coeffs_[j][2] * data[i] - this->coeffs_[j][4] * yi;

            data[i] = yi;
        }
    }
}

void SOS_Filter::reset()
{
    for (auto &s : this->state_)
        s = {0.f, 0.f};
}

} // namespace sound_level_meter
} // namespace esphome
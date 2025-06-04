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
#ifdef USE_I2S_LEGACY
    ESP_LOGCONFIG(TAG, "  Bits Per Sample %u", this->bits_per_sample_);
#else
    uint8_t bits_per_sample = 16;
    if (this->slot_bit_width_ != I2S_SLOT_BIT_WIDTH_AUTO) {
    bits_per_sample = this->slot_bit_width_;
    }
    ESP_LOGCONFIG(TAG, "  Bits Per Sample %u", bits_per_sample);
#endif
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
    esp_err_t err;

#ifdef USE_I2S_LEGACY
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
#else
    i2s_chan_config_t chan_cfg = {
        .id = this->parent_->get_port(),
        .role = this->i2s_role_,
        .dma_desc_num = 4,
        .dma_frame_num = 256,
        .auto_clear = false,
    };
    /* Allocate a new RX channel and get the handle of this channel */
    err = i2s_new_channel(&chan_cfg, NULL, &this->rx_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error creating new I2S channel: %s", esp_err_to_name(err));
        this->status_set_error();
        return;
    }

    i2s_clock_src_t clk_src = I2S_CLK_SRC_DEFAULT;
#ifdef I2S_CLK_SRC_APLL
    if (this->use_apll_) {
        clk_src = I2S_CLK_SRC_APLL;
    }
#endif
    i2s_std_gpio_config_t pin_config = this->parent_->get_pin_config();

    {
        i2s_std_clk_config_t clk_cfg = {
            .sample_rate_hz = this->sample_rate_,
            .clk_src = clk_src,
            .mclk_multiple = I2S_MCLK_MULTIPLE_384,
        };
        i2s_std_slot_config_t std_slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG((i2s_data_bit_width_t) this->slot_bit_width_, this->slot_mode_);
        std_slot_cfg.slot_bit_width = this->slot_bit_width_;
        std_slot_cfg.slot_mask = this->std_slot_mask_;
    
        pin_config.din = this->din_pin_;
    
        i2s_std_config_t std_cfg = {
            .clk_cfg = clk_cfg,
            .slot_cfg = std_slot_cfg,
            .gpio_cfg = pin_config,
        };
        /* Initialize the channel */
        err = i2s_channel_init_std_mode(this->rx_handle_, &std_cfg);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error initializing I2S channel: %s", esp_err_to_name(err));
        this->status_set_error();
        return;
    }

    /* Before reading data, start the RX channel first */
    i2s_channel_enable(this->rx_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error enabling I2S Microphone: %s", esp_err_to_name(err));
        this->status_set_error();
        return;
    }
#endif

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
    if (this->publish_queue_ == nullptr) {
        ESP_LOGE(TAG, "Publish queue is not initialized!");
        return;
    }

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
    TickType_t ticks_to_wait = 100 / portTICK_PERIOD_MS;

    // Perform an I2S read with up to a 100ms timeout
#ifdef USE_I2S_LEGACY
    esp_err_t err = i2s_read(
        this->parent_->get_port(),  // which I2S port
        buf,                        // destination buffer (int32_t*)
        bytes_to_read,             // how many bytes we want
        &bytes_read,               // how many bytes we actually got
        ticks_to_wait
    );
#else
    // i2s_channel_read expects the timeout value in ms, not ticks
    esp_err_t err = i2s_channel_read(
        this->rx_handle_, 
        buf, 
        bytes_to_read, 
        &bytes_read, 
        100);
#endif

    // Handle errors or zero data
    if ((err != ESP_OK) && ((err != ESP_ERR_TIMEOUT) || (ticks_to_wait != 0))) {
        // Ignore ESP_ERR_TIMEOUT if ticks_to_wait = 0, as it will read the data on the next call
        if (!this->status_has_warning()) {
            // Avoid spamming the logs with the error message if its repeated
            ESP_LOGW(TAG, "Error reading from I2S microphone: %s", esp_err_to_name(err));
        }
        this->status_set_warning();
        return 0;
    }
    if ((bytes_read == 0) && (ticks_to_wait > 0)) {
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
    if (!this->filters_.empty()) {
        for (auto f : this->filters_) {
            f->process(buffer);
        }
    }

    for (auto s : this->sensors_) {
        s->process(buffer);
    }

    for (auto g : this->groups_) {
        g->process(buffer);
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
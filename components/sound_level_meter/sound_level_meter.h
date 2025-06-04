#pragma once

#include <mutex>
#include <condition_variable>
#include <algorithm>
#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2s_audio/i2s_audio.h"
#include <freertos/queue.h>

namespace esphome {
namespace sound_level_meter {
class SensorGroup;
class SoundLevelMeterSensor;
class Filter;

class SoundLevelMeter : public i2s_audio::I2SAudioIn, public Component {
  friend class SoundLevelMeterSensor;
  public:
    void setup() override;
    void dump_config() override;
    void loop() override;
  
#ifdef USE_I2S_LEGACY
    void set_din_pin(int8_t pin) { this->din_pin_ = pin; }
#else
    void set_din_pin(int8_t pin) { this->din_pin_ = (gpio_num_t) pin; }
#endif
    void set_pdm(bool pdm) { this->pdm_ = pdm; }
    void set_update_interval(uint32_t update_interval) {this->update_interval_ = update_interval;}
    uint32_t get_update_interval() { return this->update_interval_; }
    void add_group(SensorGroup *group) { this->groups_.push_back(group); }
    void set_warmup_interval(uint32_t warmup_interval) { this->warmup_interval_ = warmup_interval; }
    void set_mic_sensitivity(optional<float> mic_sensitivity) { this->mic_sensitivity_ = mic_sensitivity; }
    optional<float> get_mic_sensitivity() { return this->mic_sensitivity_; }
    void set_mic_sensitivity_ref(optional<float> mic_sensitivity_ref) { this->mic_sensitivity_ref_ = mic_sensitivity_ref; }
    optional<float> get_mic_sensitivity_ref(){ return this->mic_sensitivity_ref_; }
    void set_offset(optional<float> offset) { this->offset_ = offset; }
    optional<float> get_offset() { return this->offset_; }
    void set_max_groups_depth(uint8_t max_groups_depth) { this->max_groups_depth_ = max_groups_depth; }
    uint32_t get_sample_rate();

  protected:
    size_t read_(int32_t *buf, size_t frames_requested, std::vector<float> &buffer);
  
#ifdef USE_I2S_LEGACY
    int8_t din_pin_{I2S_PIN_NO_CHANGE};
#else
    gpio_num_t din_pin_{I2S_GPIO_UNUSED};
    i2s_chan_handle_t rx_handle_;
#endif
    bool pdm_{false};

    std::vector<SensorGroup *> groups_;
    uint32_t warmup_interval_{500};
    uint32_t warmup_start_{0};
    optional<float> mic_sensitivity_{};
    optional<float> mic_sensitivity_ref_{};
    optional<float> offset_{};
    uint32_t update_interval_{60000};
    uint8_t max_groups_depth_{1};
    QueueHandle_t publish_queue_;
  
    static void task(void *param);
    void reset();
};

class SensorGroup {
 public:
  void set_parent(SoundLevelMeter *parent);
  void add_sensor(SoundLevelMeterSensor *sensor);
  void add_group(SensorGroup *group);
  void add_filter(Filter *filter);
  void process(std::vector<float> &buffer);
  void dump_config(const char *prefix);
  void reset();

 protected:
  SoundLevelMeter *parent_{nullptr};
  std::vector<SensorGroup *> groups_;
  std::vector<SoundLevelMeterSensor *> sensors_;
  std::vector<Filter *> filters_;
};

class SoundLevelMeterSensor : public sensor::Sensor {
  friend class SensorGroup;

 public:
  void set_parent(SoundLevelMeter *parent);
  void set_update_interval(uint32_t update_interval);
  virtual void process(std::vector<float> &buffer) = 0;
  void queue_publish_state(float state);

 protected:
  SoundLevelMeter *parent_{nullptr};
  uint32_t update_samples_{0};
  float adjust_dB(float dB, bool is_rms = true);

  virtual void reset() = 0;
};

class SoundLevelMeterSensorEq : public SoundLevelMeterSensor {
 public:
  virtual void process(std::vector<float> &buffer) override;

 protected:
  double sum_{0.};
  uint32_t count_{0};

  virtual void reset() override;
};

class SoundLevelMeterSensorMax : public SoundLevelMeterSensor {
 public:
  void set_window_size(uint32_t window_size);
  virtual void process(std::vector<float> &buffer) override;

 protected:
  uint32_t window_samples_{0};
  float sum_{0.f};
  float max_{std::numeric_limits<float>::min()};
  uint32_t count_sum_{0}, count_max_{0};

  virtual void reset() override;
};

class SoundLevelMeterSensorMin : public SoundLevelMeterSensor {
 public:
  void set_window_size(uint32_t window_size);
  virtual void process(std::vector<float> &buffer) override;

 protected:
  uint32_t window_samples_{0};
  float sum_{0.f};
  float min_{std::numeric_limits<float>::max()};
  uint32_t count_sum_{0}, count_min_{0};

  virtual void reset() override;
};

class SoundLevelMeterSensorPeak : public SoundLevelMeterSensor {
 public:
  virtual void process(std::vector<float> &buffer) override;

 protected:
  float peak_{0.f};
  uint32_t count_{0};

  virtual void reset() override;
};

class Filter {
  friend class SensorGroup;

 public:
  virtual void process(std::vector<float> &data) = 0;

 protected:
  virtual void reset() = 0;
};

class SOS_Filter : public Filter {
 public:
  SOS_Filter(std::initializer_list<std::initializer_list<float>> &&coeffs);
  virtual void process(std::vector<float> &data) override;

 protected:
  std::vector<std::array<float, 5>> coeffs_;  // {b0, b1, b2, a1, a2}
  std::vector<std::array<float, 2>> state_;

  virtual void reset() override;
};

}   // namespace sound_level_meter
}  // namespace esphome
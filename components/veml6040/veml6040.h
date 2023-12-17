#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome
{
  namespace veml6040
  {

    enum VEML6040IntegrationTime
    {
      VEML6040_INTEGRATION_TIME_40MS = 0x00,
      VEML6040_INTEGRATION_TIME_80MS = 0x01,
      VEML6040_INTEGRATION_TIME_160MS = 0x02,
      VEML6040_INTEGRATION_TIME_320MS = 0x03,
      VEML6040_INTEGRATION_TIME_640MS = 0x04,
      VEML6040_INTEGRATION_TIME_1280MS = 0x05,
    };

    class VEML6040Component : public PollingComponent, public i2c::I2CDevice
    {
    public:
      void set_integration_time(VEML6040IntegrationTime integration_time);
      void set_glass_attenuation_factor(float ga);

      void set_illuminance_sensor(sensor::Sensor *illuminance_sensor) { illuminance_sensor_ = illuminance_sensor; }
      void set_color_temperature_sensor(sensor::Sensor *color_temperature_sensor)
      {
        color_temperature_sensor_ = color_temperature_sensor;
      }

      void setup() override;
      float get_setup_priority() const override;
      void update() override;
      void dump_config() override;

    protected:
      i2c::ErrorCode read_data_register_(uint8_t a_register, uint16_t &data)
      {
        uint8_t buffer[2];
        auto retval = this->read_register(a_register, buffer, 2);
        if (retval == i2c::ERROR_OK)
          data = (uint16_t(buffer[1]) << 8) | (uint16_t(buffer[0]) & 0xFF);
        return retval;
      }
      i2c::ErrorCode write_config_register_(uint8_t a_register, uint8_t data)
      {
        return this->write_register(a_register, &data, 1);
      }
      sensor::Sensor *illuminance_sensor_{nullptr};
      sensor::Sensor *color_temperature_sensor_{nullptr};
      float sensitivity{0.12584};
      float glass_attenuation_{0.5};
      float illuminance_;
      float color_temperature_;

    private:
      void calculate_temperature_and_lux_(uint16_t r, uint16_t g, uint16_t b, uint16_t c);
      uint16_t integration_reg_;
    };
  }
}
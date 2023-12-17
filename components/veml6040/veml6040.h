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

    // // VEML6040 I2C ADDRESS
    // #define VEML6040_I2C_ADDRESS   0x10

    // // REGISTER CONF (00H) SETTINGS

    // #define VEML6040_IT_40MS       0x00
    // #define VEML6040_IT_80MS       0x10
    // #define VEML6040_IT_160MS      0x20
    // #define VEML6040_IT_320MS      0x30
    // #define VEML6040_IT_640MS      0x40
    // #define VEML6040_IT_1280MS     0x50

    // #define VEML6040_TRIG_DISABLE  0x00
    // #define VEML6040_TRIG_ENABLE   0x04

    // #define VEML6040_AF_AUTO       0x00
    // #define VEML6040_AF_FORCE      0x02

    // #define VEML6040_SD_ENABLE     0x00
    // #define VEML6040_SD_DISABLE    0x01

    // // COMMAND CODES

    // #define COMMAND_CODE_CONF      0x00
    // #define COMMAND_CODE_RED       0x08
    // #define COMMAND_CODE_GREEN     0x09
    // #define COMMAND_CODE_BLUE      0x0A
    // #define COMMAND_CODE_WHITE     0x0B

    // // G SENSITIVITY

    // #define VEML6040_GSENS_40MS       0.25168
    // #define VEML6040_GSENS_80MS       0.12584
    // #define VEML6040_GSENS_160MS      0.06292
    // #define VEML6040_GSENS_320MS      0.03146
    // #define VEML6040_GSENS_640MS      0.01573
    // #define VEML6040_GSENS_1280MS     0.007865

    // class VEML6040CustomSensor : public PollingComponent, public i2c::I2CDevice {
    //  public:
    //   // constructor
    //   VEML6040CustomSensor() : PollingComponent(15000) {}

    //   Sensor *ambient_light = new Sensor();
    //   Sensor *color_temp = new Sensor();

    // float get_setup_priority() const override { return esphome::setup_priority::BUS; }

    //   void setup() override {
    //     Wire.begin();

    //     Wire.beginTransmission(VEML6040_I2C_ADDRESS);
    //     Wire.write(COMMAND_CODE_CONF);
    //     Wire.write(VEML6040_IT_80MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);
    //     Wire.write(0);
    //     Wire.endTransmission();
    //   }

    //   void update() override {
    //     float ambientLight = getAmbientLight();
    //     ambient_light->publish_state(ambientLight);

    //     uint16_t cct = getCCT(0.5);
    //     color_temp->publish_state(cct);
    //   }
    // private:

    // uint16_t read(uint8_t commandCode) {
    //   uint16_t data = 0;

    //   Wire.beginTransmission(VEML6040_I2C_ADDRESS);
    //   Wire.write(commandCode);
    //   Wire.endTransmission(false);
    //   Wire.requestFrom(VEML6040_I2C_ADDRESS,2);
    //   while(Wire.available())
    //   {
    //     data = Wire.read();
    //     data |= Wire.read() << 8;
    //   }

    //   return data;
    // }

    // uint16_t getRed(void) {
    //   return(read(COMMAND_CODE_RED));
    // }

    // uint16_t getGreen(void) {
    //   return(read(COMMAND_CODE_GREEN));
    // }

    // uint16_t getBlue(void) {
    //   return(read(COMMAND_CODE_BLUE));
    // }

    // uint16_t getWhite(void) {
    //   return(read(COMMAND_CODE_WHITE));
    // }

    // float getAmbientLight(void) {
    //   uint16_t sensorValue;
    //   float ambientLightInLux;

    //   sensorValue = read(COMMAND_CODE_GREEN);

    // //   switch(lastConfiguration & 0x70) {

    // //     case VEML6040_IT_40MS:    ambientLightInLux = sensorValue * VEML6040_GSENS_40MS;
    // //                               break;
    // //     case VEML6040_IT_80MS:    ambientLightInLux = sensorValue * VEML6040_GSENS_80MS;
    // //                               break;
    // //     case VEML6040_IT_160MS:   ambientLightInLux = sensorValue * VEML6040_GSENS_160MS;
    // //                               break;
    //     //case VEML6040_IT_320MS:
    //     ambientLightInLux = sensorValue * VEML6040_GSENS_80MS;
    // //                               break;
    // //     case VEML6040_IT_640MS:   ambientLightInLux = sensorValue * VEML6040_GSENS_640MS;
    // //                               break;
    // //     case VEML6040_IT_1280MS:  ambientLightInLux = sensorValue * VEML6040_GSENS_1280MS;
    // //                               break;
    // //     default:                  ambientLightInLux = -1;
    // //                               break;
    // //   }
    //   return ambientLightInLux;
    // }

    // uint16_t getCCT(float offset) {
    //   uint16_t red,blue,green;
    //   float cct,ccti;

    //   red = read(COMMAND_CODE_RED);
    //   green = read(COMMAND_CODE_GREEN);
    //   blue = read(COMMAND_CODE_BLUE);

    //   ccti = ((float)red-(float)blue) / (float)green;
    //   ccti = ccti + offset;
    //   cct = 4278.6 * pow(ccti,-1.2455);

    //   return((uint16_t)cct);
    // }

    // };
  }
}
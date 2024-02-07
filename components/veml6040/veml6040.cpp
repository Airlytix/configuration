#include "veml6040.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome
{
    namespace veml6040
    {

        static const char *const TAG = "veml6040";

        static const uint8_t VEML6040_ADDRESS = 0x10;

        static const uint8_t VEML6040_TRIG_DISABLE = 0x00;
        static const uint8_t VEML6040_TRIG_ENABLE = 0x04;
        static const uint8_t VEML6040_AF_AUTO = 0x00;
        static const uint8_t VEML6040_AF_FORCE = 0x02;
        static const uint8_t VEML6040_SD_ENABLE = 0x00;
        static const uint8_t VEML6040_SD_DISABLE = 0x01;
        static const uint8_t VEML6040_REGISTER_CONTROL = 0x00;
        static const uint8_t VEML6040_REGISTER_RDATAL = 0x08;
        static const uint8_t VEML6040_REGISTER_GDATAL = 0x09;
        static const uint8_t VEML6040_REGISTER_BDATAL = 0x0A;
        static const uint8_t VEML6040_REGISTER_WDATAL = 0x0B;

        // G SENSITIVITY

        static const float VEML6040_GSENS_40MS = 0.25168;
        static const float VEML6040_GSENS_80MS = 0.12584;
        static const float VEML6040_GSENS_160MS = 0.06292;
        static const float VEML6040_GSENS_320MS = 0.03146;
        static const float VEML6040_GSENS_640MS = 0.01573;
        static const float VEML6040_GSENS_1280MS = 0.007865;

        void VEML6040Component::setup()
        {
            ESP_LOGCONFIG(TAG, "Setting up VEML6040...");

            uint8_t conf = 0;
            conf |= (this->integration_reg_ << 4);
            conf |= (VEML6040_TRIG_DISABLE | VEML6040_AF_AUTO | VEML6040_SD_ENABLE);

            uint8_t config_data[2]{conf, 0x00};

            if (this->write_config_register_(VEML6040_REGISTER_CONTROL, config_data, 2) != i2c::ERROR_OK)
            {
                this->mark_failed();
                return;
            }
        }

        void VEML6040Component::dump_config()
        {
            ESP_LOGCONFIG(TAG, "VEML6040:");
            LOG_I2C_DEVICE(this);
            if (this->is_failed())
            {
                ESP_LOGE(TAG, "Communication with VEML6040 failed!");
            }
            LOG_UPDATE_INTERVAL(this);

            LOG_SENSOR("  ", "Illuminance", this->illuminance_sensor_);
            LOG_SENSOR("  ", "Color Temperature", this->color_temperature_sensor_);
        }
        float VEML6040Component::get_setup_priority() const { return setup_priority::DATA; }

        void VEML6040Component::calculate_temperature_and_lux_(uint16_t r, uint16_t g, uint16_t b, uint16_t w)
        {
            this->illuminance_ = 0; // Assign 0 value before calculation
            this->color_temperature_ = 0;

            const float ga = this->glass_attenuation_; // Glass Attenuation Factor

            if ( g == 0) {
                return;
            }

            this->illuminance_ = (float)g * this->sensitivity;

            float ccti_raw = (float)((r-b)/(float)g);
            float ccti = ccti_raw + ga;
            this->color_temperature_ = 4278.6 * pow(ccti, -1.2455);
        }

        void VEML6040Component::update()
        {
            uint16_t raw_w;
            uint16_t raw_r;
            uint16_t raw_g;
            uint16_t raw_b;

            if (this->read_data_register_(VEML6040_REGISTER_WDATAL, raw_w) != i2c::ERROR_OK)
            {
                this->status_set_warning();
                return;
            }
            if (this->read_data_register_(VEML6040_REGISTER_RDATAL, raw_r) != i2c::ERROR_OK)
            {
                this->status_set_warning();
                return;
            }
            if (this->read_data_register_(VEML6040_REGISTER_GDATAL, raw_g) != i2c::ERROR_OK)
            {
                this->status_set_warning();
                return;
            }
            if (this->read_data_register_(VEML6040_REGISTER_BDATAL, raw_b) != i2c::ERROR_OK)
            {
                this->status_set_warning();
                return;
            }
            ESP_LOGV(TAG, "Raw values white=%d red=%d green=%d blue=%d", raw_w, raw_r, raw_g, raw_b);

            if (this->illuminance_sensor_ || this->color_temperature_sensor_)
            {
                calculate_temperature_and_lux_(raw_r, raw_g, raw_b, raw_w);
            }

            if (raw_w < 65530)
            {
                if (this->illuminance_sensor_ != nullptr)
                    this->illuminance_sensor_->publish_state(this->illuminance_);

                if (this->color_temperature_sensor_ != nullptr)
                    this->color_temperature_sensor_->publish_state(this->color_temperature_);
            }

            ESP_LOGD(TAG, "Illuminance=%.1flx Color Temperature=%.1fK", this->illuminance_, this->color_temperature_);

            this->status_clear_warning();
        }

        void VEML6040Component::set_integration_time(VEML6040IntegrationTime integration_time)
        {
            this->integration_reg_ = integration_time;
            switch (this->integration_reg_)
            {
            case VEML6040_INTEGRATION_TIME_40MS:
                this->sensitivity = VEML6040_GSENS_40MS;
                break;

            case VEML6040_INTEGRATION_TIME_80MS:
                this->sensitivity = VEML6040_GSENS_80MS;
                break;

            case VEML6040_INTEGRATION_TIME_160MS:
                this->sensitivity = VEML6040_GSENS_160MS;
                break;

            case VEML6040_INTEGRATION_TIME_320MS:
                this->sensitivity = VEML6040_GSENS_320MS;
                break;

            case VEML6040_INTEGRATION_TIME_640MS:
                this->sensitivity = VEML6040_GSENS_640MS;
                break;

            case VEML6040_INTEGRATION_TIME_1280MS:
                this->sensitivity = VEML6040_GSENS_1280MS;
                break;
            }
        }

        void VEML6040Component::set_glass_attenuation_factor(float ga)
        {
            // The Glass Attenuation (FA) factor used to compensate for lower light
            // levels at the device due to the possible presence of glass. The GA is
            // the inverse of the glass transmissivity (T), so GA = 1/T. A transmissivity
            // of 50% gives GA = 1 / 0.50 = 2. If no glass is present, use GA = 1.
            // See Application Note: DN40-Rev 1.0
            this->glass_attenuation_ = ga;
        }

    }
}
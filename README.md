# Airlytix

Airlytix is a environment monitoring solution based on the ESPHome platform. It leverages the power of various sensors to provide comprehensive data about your surroundings. This includes air quality, temperature, humidity, ambient light, and noise levels.

This repository contains the configuration files needed to set up and run Airlytix on various hardware platforms. These files are specifically designed for use with Airlytix hardware, ensuring optimal performance and reliability.

## Official Documentation

For comprehensive instructions on how to set up and use your Airlytix hardware, please refer to the [official Airlytix documentation](https://docs.airlytix.io).

## Local Development

If you're interested in local development, you can use the configuration files in this repository. Follow the detailed steps below:

1. **Clone the Repository**: Use the `git clone` command to clone this repository to your local machine.

```bash
git clone https://github.com/airlytix/configuration.git
```

2. **Install ESPHome**: If you haven't already, install ESPHome on your machine. Detailed instructions on how to do this can be found on the [ESPHome website](https://esphome.io/guides/getting_started_command_line.html)..

3. **Select the Appropriate Configuration File**: In the directory where you cloned the repository, you will find multiple configuration files for different Airlytix hardware. Select the configuration file that matches your specific hardware.

4. **Configure and Upload the Firmware to Your Airlytix Hardware**: Run ESPHome and point it to the configuration file you selected. ESPHome will compile the configuration file into a firmware binary and upload it to your Airlytix hardware.

```bash
cd airlytix
esphome your_config.yaml run
```

Replace `your_config.yaml` with the name of the configuration file you selected.

Note: For more detailed information on how to use ESPHome and these configuration files, please refer to the official [ESPHome documentation](https://esphome.io/guides/getting_started_command_line.html).

# Support
If you encounter any issues while using this repository, please open an issue on GitHub.

# Contributing
Contributions to this repository are welcome. Please open a pull request with your changes.


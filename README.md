# ESP-BME680

This is a simple wrapper for the [Bosch BME680 driver](https://github.com/BoschSensortec/BME680_driver) using Espressif [esp-idf](https://github.com/espressif/esp-idf). This way you can easily include it as a component in your project.

# Example

There's currently one example in the example directory. In the top of the file there are a couple of defines used for configuration (BME680 address, SDA, SCL pin), please change these accordingly. The example creates a task and prints measurements every 10 seconds to the console.

#include <string>

#include "odrive.hpp"
#include "log.hpp"


std::string ODRIVE_CONFIG[] = {
    "config.enable_brake_resistor 1",
    "config.brake_resistance 0.5",
    "config.dc_bus_undervoltage_trip_level 8.0",
    "config.dc_bus_overvoltage_trip_level 56.0",
    "config.dc_max_positive_current 20.0",
    "config.dc_max_negative_current -3.0",
    "config.max_regen_current 0",
    "config.gpio7_mode 0", // Set pin to digital
    "config.gpio8_mode 0," // Set pin to digital

    "axis0.encoder.config.abs_spi_cs_gpio_pin 7", // Set pin to SPI mode
    "axis1.encoder.config.abs_spi_cs_gpio_pin 8", // Set pin to SPI mode
};


bool ODrive::init() {
    // ensure serial is clear
    serial.write("\n");
    serial.readStringUntil('\n');
    serial.flush();

    // check if connected
    int fw_v_m = send("r fw_version_minor").toInt();

    if(fw_v_m != 5) {
        Log("Failed to connect to odrive\n");
        return false;
    }

    String sn = send("r serial_number");

    Log("Connected to SN: %s\n", sn.substring(0, sn.length() - 1).c_str());

    serial.setTimeout(5);

    for (size_t i = 0; i < sizeof(ODRIVE_CONFIG) / sizeof(ODRIVE_CONFIG[0]); ++i) {
        String resp = send("w %s", ODRIVE_CONFIG[i].c_str());
        if(resp.length() > 0) { // the odrive will only respond if the command fails
            Log("w %s reponded: %s\n", ODRIVE_CONFIG[i].c_str(), resp.c_str());
        }
    }

    // init axis
    axis0.init();
    axis1.init();

    send("ss"); // save config

    return true;
}

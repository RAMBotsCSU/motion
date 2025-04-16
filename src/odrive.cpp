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


void ODrive::init() {
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

    _initialized = true;
}

void ODrive::connect() {
    serial.setTimeout(100);

    // ensure serial is clear
    // can't get serial.flush() to work but this does
    while(serial.available()) {
        serial.readStringUntil('\n');
    }

    // ensure write buffer is clear
    serial.print("foo\n");
    serial.readStringUntil('\n');

    // check if connected
    int fw_v_m = send("r fw_version_minor").toInt();

    if(fw_v_m != 5) {
        Log("Failed to connect to odrive\n");
        return;
    }

    int sn = send("r serial_number").toInt();

    // Happens sometimes for some reason
    if(sn == 0) return;

    // the odrive doesnt sen the full sn for some reason
    // so this annoying math reconstructs most of it.
    Log("Connected to SN: 0x%" PRIX64 "\n", (uint64_t)sn * 100000ULL >> 24);

    _connected = true;

    serial.setTimeout(5);

    // only run this on first connection.
    if(!isInitialized()) {
        init();
    }
}
}

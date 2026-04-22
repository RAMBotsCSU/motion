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
    "config.gpio8_mode 0", // Set pin to digital

    "axis0.encoder.config.abs_spi_cs_gpio_pin 7", // Set pin to SPI mode
    "axis1.encoder.config.abs_spi_cs_gpio_pin 8", // Set pin to SPI mode
    "axis0.encoder.config.mode 257",
    "axis1.encoder.config.mode 257"
};


void ODrive::init() {
    // Check axis0 only — both axes share the same ODrive so their state matches.
    // If already in closed-loop the ODrive was running before the Teensy connected.
    // Skip encoder/GPIO config writes that would reinitialize the encoder and
    // drop the axes out of closed-loop.
    bool alreadyRunning = (axis0.fetchState() == AXIS_STATE_CLOSED_LOOP_CONTROL);

    if (!alreadyRunning) {
        for (size_t i = 0; i < sizeof(ODRIVE_CONFIG) / sizeof(ODRIVE_CONFIG[0]); ++i) {
            String resp = send("w %s", ODRIVE_CONFIG[i].c_str());
            if(resp.length() > 0) {
                Log("w %s reponded: %s\n", ODRIVE_CONFIG[i].c_str(), resp.c_str());
            }
        }
    } else {
        Log("ODrive already running, skipping top-level config\n");
    }

    axis0.init(alreadyRunning);
    axis1.init(alreadyRunning);

    if (!alreadyRunning) {
        send("ss"); // save config
    }

    _initialized = true;
}

void ODrive::connect() {
    serial.setTimeout(100);

    // Drain anything already in the buffer
    while(serial.available()) {
        serial.readStringUntil('\n');
    }

    // Send a dummy command to provoke any stale ODrive responses, then wait
    // for them all to arrive before draining again. Without this delay, stale
    // responses queued while the Teensy was off can overlap with the
    // fw_version_minor response and corrupt the read.
    serial.print("foo\n");
    delay(200);
    while(serial.available()) {
        serial.readStringUntil('\n');
    }

    // check if connected
    int fw_v_m = send("r fw_version_minor").toInt();

    if(fw_v_m < 4 || fw_v_m > 9) {
        Log("Failed to connect to odrive (fw_version_minor=%d)\n", fw_v_m);
        return;
    }

    Log("Connected to ODrive (fw_version_minor=%d)\n", fw_v_m);

    _connected = true;

    serial.setTimeout(5);

    init();
}

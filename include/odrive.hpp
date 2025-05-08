#ifndef ODRIVE_H
#define ODRIVE_H

#include <HardwareSerial.h>
#include <string>

#include "axis.hpp"
#include "log.hpp"

// Helper functions for checksum calculation and validation.
static unsigned int calculateChecksum(const std::string &data) {
    unsigned int checksum = 0;
    for (char ch : data) {
        checksum ^= static_cast<unsigned char>(ch);
    }
    return checksum;
}

static bool validateChecksum(const std::string &line) {
    size_t starPos = line.find('*');
    if(starPos == std::string::npos) {
        // No checksum provided, assume valid.
        return true;
    }
    std::string dataPart = line.substr(0, starPos);
    // Extract provided checksum
    size_t pos = starPos + 1;
    std::string checksumStr;
    while(pos < line.size() && std::isdigit(line[pos])) {
        checksumStr.push_back(line[pos]);
        ++pos;
    }
    if(checksumStr.empty()) return false;
    unsigned int providedChecksum = std::stoul(checksumStr);
    unsigned int calc = calculateChecksum(dataPart);
    return (calc == providedChecksum);
}

class ODrive {
private:
    HardwareSerial& serial;
    int _sn = 0;
    bool _connected = false;
    bool _initialized = false;
    unsigned long _last_send = 0;

public:
    ODrive(HardwareSerial& _serial) : serial(_serial), axis0(*this, 0), axis1(*this, 1) {}

    void init(void);
    void connect(void);

    bool isConnected(void) {
        return _connected;
    }

    bool isInitialized(void) {
        return _initialized;
    }

    String send(const char* format, ...) {
        va_list args;
        va_start(args, format);
        va_list args_copy;
        va_copy(args_copy, args);
        int len = vsnprintf(nullptr, 0, format, args_copy);
        va_end(args_copy);
        std::string s(len, '\0');
        vsnprintf(&s[0], len + 1, format, args);
        va_end(args);

        // Add space before checksum
        s.push_back(' ');

        // Calculate checksum over the modified string.
        unsigned int cs = calculateChecksum(s);
        s += "*" + std::to_string(cs) + "\n";

        // Log("%s", s.c_str());

        if (s[0] != 't') {
            while(millis() - _last_send < 15); // rate limit to avoid making the odrive sad
            _last_send = millis();
        }

        serial.print(s.c_str());

        // Only read response if command starts with "r " or "w "
        if(!(s.size() >= 2 && ((s[0]=='r' && s[1]==' ') || (s[0]=='w' && s[1]==' ')))) {
            return "";
        }

        String resp = serial.readStringUntil('\n');
        std::string respStr(resp.c_str());

        // Log("===== %d %c\n", respStr.size(), s[0]);

        // If read command and no response
        if(isConnected() && respStr.size() == 0 && s[0]=='r') {
            Log("Lost connection to odrive 0x%X\n", _sn);
            _connected = false;
        }

        // Validate the checksum in the response if present.
        if(!validateChecksum(respStr)) {
            Log("Invalid checksum in response: %s\n", respStr.c_str());
            return "";
        }

        // Remove checksum data from the response.
        size_t starPos = respStr.find('*');
        if (starPos != std::string::npos) {
            respStr = respStr.substr(0, starPos);
        }

        // Log("%s\n", respStr.c_str());

        return String(respStr.c_str());
    }

    Axis axis0;
    Axis axis1;
};

#endif  // ODRIVE_H

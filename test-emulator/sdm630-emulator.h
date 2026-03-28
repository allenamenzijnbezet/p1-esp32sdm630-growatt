#include "esphome.h"

unsigned long last_command_time = 0;

uint16_t modbus_crc(uint8_t *buf, int len) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// Returns the float value for a given SDM630 register address (even addresses only)
float get_register_value(uint16_t reg, float watt, float voltage) {
    float p_phase   = watt / 3.0f;
    float a_phase   = (voltage > 0) ? fabsf(p_phase) / voltage : 0.0f;
    float va_phase  = voltage * a_phase;
    float var_phase = sqrtf(fmaxf(0.0f, va_phase*va_phase - p_phase*p_phase));
    float pf_phase  = (va_phase > 0.001f) ? p_phase / va_phase : 0.0f;
    float angle_ph  = (fabsf(pf_phase) <= 1.0f) ? acosf(fabsf(pf_phase)) * 57.2958f : 0.0f;
    float va_total  = va_phase  * 3.0f;
    float var_total = var_phase * 3.0f;
    float pf_total  = (va_total > 0.001f) ? watt / va_total : 0.0f;
    float v_ll      = voltage * 1.73205f;

    switch (reg) {
        // Voltage L-N per phase
        case 0x0000: case 0x0002: case 0x0004: return voltage;
        // Current per phase
        case 0x0006: case 0x0008: case 0x000A: return a_phase;
        // Active power per phase (signed)
        case 0x000C: case 0x000E: case 0x0010: return p_phase;
        // Apparent power VA per phase
        case 0x0012: case 0x0014: case 0x0016: return va_phase;
        // Reactive power VAr per phase
        case 0x0018: case 0x001A: case 0x001C: return var_phase;
        // Power factor per phase (signed)
        case 0x001E: case 0x0020: case 0x0022: return pf_phase;
        // Phase angle per phase (degrees)
        case 0x0024: case 0x0026: case 0x0028: return angle_ph;
        // Average L-N voltage
        case 0x002A: return voltage;
        // Average line current
        case 0x002E: return a_phase;
        // Sum of line currents
        case 0x0030: return a_phase * 3.0f;
        // Total active power (signed W)
        case 0x0034: return watt;
        // Total VA
        case 0x0038: return va_total;
        // Total VAr
        case 0x003C: return var_total;
        // Total PF
        case 0x003E: return pf_total;
        // Total phase angle
        case 0x0042: return angle_ph;
        // Frequency
        case 0x0046: return 50.0f;
        // Energy counters (zero in this simple version)
        case 0x0048: case 0x004A: case 0x004C: case 0x004E:
        case 0x0050: case 0x0052: return 0.0f;
        // Line-to-line voltages
        case 0x00C8: case 0x00CA: case 0x00CC: case 0x00CE: return v_ll;
        // Total kWh / kVArh
        case 0x0156: case 0x0158: return 0.0f;
        // Everything else
        default: return 0.0f;
    }
}

void process_modbus() {
    static uint8_t request[8];
    static int idx = 0;
    static unsigned long last_byte_time = 0;

    // 1. Accumulate bytes across calls
    while (id(modbus_bus).available() > 0 && idx < 8) {
        id(modbus_bus).read_byte(&request[idx++]);
        last_byte_time = millis();
    }

    // 2. Wait for 8 bytes AND 4ms silence = complete frame
    if (idx < 8 || (millis() - last_byte_time) < 4) return;

    // 3. Log incoming frame
    char hex_log[25];
    int pos = 0;
    for (int i = 0; i < 8 && pos < 24; i++)
        pos += snprintf(hex_log + pos, sizeof(hex_log) - pos, "%02X ", request[i]);
    ESP_LOGD("modbus_rx", "Binnen: %s", hex_log);

    // 4. Reset for next frame
    idx = 0;

    // 5. Check address, function code and CRC
    if (request[0] != 0x02) return;  // Not our address
    if (request[1] != 0x04) return;  // Only FC04 supported

    uint16_t crc_calc = modbus_crc(request, 6);
    uint16_t crc_rx   = request[6] | (request[7] << 8);
    if (crc_calc != crc_rx) {
        ESP_LOGW("modbus_rx", "CRC fout!");
        return;
    }

    last_command_time = millis();

    uint16_t start_reg = (request[2] << 8) | request[3];
    uint16_t reg_count = (request[4] << 8) | request[5];

    float watt    = id(manual_wattage).state;
    float voltage = 236.0f;

    // 6. Build response: 3 header bytes + reg_count*2 data bytes + 2 CRC bytes
    uint8_t response[256];
    response[0] = 0x02;           // Slave address
    response[1] = 0x04;           // Function code
    response[2] = reg_count * 2;  // Byte count

    int offset = 3;
    for (uint16_t r = 0; r < reg_count; r += 2) {
        uint16_t reg = start_reg + r;
        float val = get_register_value(reg, watt, voltage);

        uint32_t raw;
        memcpy(&raw, &val, sizeof(float));
        response[offset++] = (raw >> 24) & 0xFF;
        response[offset++] = (raw >> 16) & 0xFF;
        response[offset++] = (raw >>  8) & 0xFF;
        response[offset++] =  raw        & 0xFF;
    }

    uint16_t crc = modbus_crc(response, offset);
    response[offset++] = crc & 0xFF;
    response[offset++] = (crc >> 8) & 0xFF;

    for (int i = 0; i < offset; i++)
        id(modbus_bus).write_byte(response[i]);

    // 7. Log response
    std::string hex_str = "";
    for (int i = 0; i < offset; i++) {
        char buf[4];
        snprintf(buf, sizeof(buf), "%02X ", response[i]);
        hex_str += buf;
    }
    ESP_LOGD("modbus_tx", "Verzonden HEX: %s", hex_str.c_str());
    ESP_LOGI("modbus_tx", "Antwoord voor reg 0x%04X: %.1f Watt", start_reg, watt);

    // 8. Flush any leftover bytes
    uint8_t dummy;
    while (id(modbus_bus).available() > 0)
        id(modbus_bus).read_byte(&dummy);
}

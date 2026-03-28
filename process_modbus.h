#pragma once
#include "esphome.h"

// ═══════════════════════════════════════════════════════════════════════════
// process_modbus.h
// SDM630 Modbus RTU emulator for ESPHome
//
// This code runs on every loop() call. It:
//   1. Reads incoming Modbus RTU requests from the Growatt inverter
//   2. Gets real grid data from the P1 smart meter (via ESPHome DSMR sensors)
//   3. Maps that data to SDM630 register addresses
//   4. Sends back a correctly formatted Modbus RTU response
//
// If P1 data is unavailable or stale, no response is sent at all.
// The Growatt will time out and stop producing — exactly like a real
// SDM630 would behave if it had a malfunction.
//
// Modbus slave address: 0x02
// Function code: FC04 (Read Input Registers)
// Register format: IEEE 754 float32, big-endian word order
// ═══════════════════════════════════════════════════════════════════════════

// ───────────────────────────────────────────────────────────────────────────
// modbus_crc()
// Calculates the CRC16 checksum for a Modbus RTU frame.
// Modbus uses polynomial 0xA001 (reversed 0x8005), init value 0xFFFF.
// The CRC is appended to every request and response frame.
// ───────────────────────────────────────────────────────────────────────────
uint16_t modbus_crc(uint8_t *buf, int len) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) { crc >>= 1; crc ^= 0xA001; }
            else                     { crc >>= 1; }
        }
    }
    return crc;
}

// ───────────────────────────────────────────────────────────────────────────
// GridData struct
// Holds the current grid measurements for all 3 phases.
// Filled from P1 DSMR sensor values.
// ───────────────────────────────────────────────────────────────────────────
struct GridData {
    float v[3];  // Voltage per phase in Volts (e.g. 236.1)
    float p[3];  // Net power per phase in Watts — positive=import, negative=export
    float i[3];  // Current per phase in Amps
};

// ───────────────────────────────────────────────────────────────────────────
// p1_valid()
// Returns true if P1 sensor data is available and valid.
// Checks that voltages are non-NaN and above 0V.
// ───────────────────────────────────────────────────────────────────────────
bool p1_valid() {
    return !isnan(id(p1_v1).state) &&
           !isnan(id(p1_v2).state) &&
           !isnan(id(p1_v3).state) &&
           !isnan(id(p1_p1_pos).state) &&
           !isnan(id(p1_p1_neg).state) &&
           id(p1_v1).state > 0 &&
           id(p1_v2).state > 0 &&
           id(p1_v3).state > 0;
}

// ───────────────────────────────────────────────────────────────────────────
// get_grid_data()
// Reads P1 sensor values and returns a GridData struct.
// P1 power values are in kW — converted to W by multiplying by 1000.
// P1 current has only 1A resolution — if it reads 0A but there is power,
// current is calculated from P/V instead.
// ───────────────────────────────────────────────────────────────────────────
GridData get_grid_data() {
    GridData d;

    // Voltages (V)
    d.v[0] = id(p1_v1).state;
    d.v[1] = id(p1_v2).state;
    d.v[2] = id(p1_v3).state;

    // Net power per phase in Watts (import - export)
    // Positive = consuming from grid, negative = feeding back to grid
    d.p[0] = (id(p1_p1_pos).state - id(p1_p1_neg).state) * 1000.0f;
    d.p[1] = (id(p1_p2_pos).state - id(p1_p2_neg).state) * 1000.0f;
    d.p[2] = (id(p1_p3_pos).state - id(p1_p3_neg).state) * 1000.0f;

    // Current: use measured value if > 0, otherwise calculate from P/V
    // P1 current has only 1A resolution so small loads show as 0A
    float i1 = id(p1_i1).state;
    float i2 = id(p1_i2).state;
    float i3 = id(p1_i3).state;
    d.i[0] = (!isnan(i1) && i1 > 0) ? i1 : fabsf(d.p[0]) / d.v[0];
    d.i[1] = (!isnan(i2) && i2 > 0) ? i2 : fabsf(d.p[1]) / d.v[1];
    d.i[2] = (!isnan(i3) && i3 > 0) ? i3 : fabsf(d.p[2]) / d.v[2];

    return d;
}

// ───────────────────────────────────────────────────────────────────────────
// get_register_value()
// Maps an SDM630 Modbus register address to a calculated float value.
//
// The SDM630 register map uses IEEE 754 float32 values at even addresses.
// Each float occupies 2 registers (4 bytes). So register 0x0000 and 0x0001
// together form one float — but we only pass even addresses here.
//
// All derived values (VA, VAr, PF, angles) are calculated from the
// measured V, I and P values.
// ───────────────────────────────────────────────────────────────────────────
float get_register_value(uint16_t reg, const GridData &g) {

    // Calculate derived values per phase
    float va[3];      // Apparent power (VA) = V * I
    float var_[3];    // Reactive power (VAr) = sqrt(VA^2 - P^2)
    float pf[3];      // Power factor = P / VA (signed: + import, - export)
    float angle[3];   // Phase angle in degrees = acos(|PF|) * 57.2958

    for (int j = 0; j < 3; j++) {
        float va_ph  = g.v[j] * g.i[j];
        float var_ph = sqrtf(fmaxf(0.0f, va_ph*va_ph - g.p[j]*g.p[j]));
        float pf_ph  = (va_ph > 0.001f) ? g.p[j] / va_ph : 0.0f;
        va[j]        = va_ph;
        var_[j]      = var_ph;
        pf[j]        = pf_ph;
        angle[j]     = (fabsf(pf_ph) <= 1.0f) ? acosf(fabsf(pf_ph)) * 57.2958f : 0.0f;
    }

    // System totals
    float total_p   = g.p[0]  + g.p[1]  + g.p[2];
    float total_va  = va[0]   + va[1]   + va[2];
    float total_var = var_[0] + var_[1] + var_[2];
    float total_pf  = (total_va > 0.001f) ? total_p / total_va : 0.0f;
    float avg_v     = (g.v[0] + g.v[1] + g.v[2]) / 3.0f;
    float avg_i     = (g.i[0] + g.i[1] + g.i[2]) / 3.0f;
    float v_ll      = avg_v * 1.73205f;  // Line-to-line voltage = V_LN * sqrt(3)

    switch (reg) {
        // Phase voltages L-N (Volts)
        case 0x0000: return g.v[0];
        case 0x0002: return g.v[1];
        case 0x0004: return g.v[2];

        // Phase currents (Amps)
        case 0x0006: return g.i[0];
        case 0x0008: return g.i[1];
        case 0x000A: return g.i[2];

        // Phase active power (Watts, signed)
        case 0x000C: return g.p[0];
        case 0x000E: return g.p[1];
        case 0x0010: return g.p[2];

        // Phase apparent power (VA)
        case 0x0012: return va[0];
        case 0x0014: return va[1];
        case 0x0016: return va[2];

        // Phase reactive power (VAr)
        case 0x0018: return var_[0];
        case 0x001A: return var_[1];
        case 0x001C: return var_[2];

        // Phase power factor (signed)
        case 0x001E: return pf[0];
        case 0x0020: return pf[1];
        case 0x0022: return pf[2];

        // Phase angles (degrees)
        case 0x0024: return angle[0];
        case 0x0026: return angle[1];
        case 0x0028: return angle[2];

        // System averages
        case 0x002A: return avg_v;
        case 0x002E: return avg_i;
        case 0x0030: return g.i[0] + g.i[1] + g.i[2];

        // System totals
        case 0x0034: return total_p;
        case 0x0038: return total_va;
        case 0x003C: return total_var;
        case 0x003E: return total_pf;
        case 0x0042: return (fabsf(total_pf) <= 1.0f) ? acosf(fabsf(total_pf)) * 57.2958f : 0.0f;

        // Frequency — not in P1, hardcoded to 50Hz
        case 0x0046: return 50.0f;

        // Energy counters — not accumulated, always 0
        case 0x0048: case 0x004A:
        case 0x004C: case 0x004E:
        case 0x0050: case 0x0052:
            return 0.0f;

        // Line-to-line voltages (V_LN * sqrt(3))
        case 0x00C8: case 0x00CA: case 0x00CC: case 0x00CE: return v_ll;

        // Total kWh / kVArh — not accumulated, always 0
        case 0x0156: case 0x0158: return 0.0f;

        // Unknown register — return 0
        default: return 0.0f;
    }
}

// ───────────────────────────────────────────────────────────────────────────
// process_modbus()
// Called every loop(). Handles one complete Modbus RTU transaction:
//
//   1. Accumulate incoming bytes into request[] buffer
//   2. Wait for 8 bytes AND 4ms silence = complete Modbus frame
//   3. Validate slave address (0x02) and function code (FC04)
//   4. Validate CRC16
//   5. Check P1 data validity — if invalid, do NOT respond (Growatt times out)
//   6. Build response: header + float32 register values + CRC
//   7. Send response back to Growatt
// ───────────────────────────────────────────────────────────────────────────
void process_modbus() {
    static uint8_t request[8];
    static int idx = 0;
    static unsigned long last_byte_time = 0;

    // Step 1: Read available bytes into buffer
    while (id(modbus_bus).available() > 0 && idx < 8) {
        id(modbus_bus).read_byte(&request[idx++]);
        last_byte_time = millis();
    }

    // Step 2: Wait for 8 bytes AND 4ms silence (Modbus inter-frame gap)
    if (idx < 8 || (millis() - last_byte_time) < 4) return;

    // Reset buffer for next frame
    idx = 0;

    // Step 3a: Check slave address — only respond to address 0x02
    if (request[0] != 0x02) return;

    // Step 3b: Check function code — only support FC04 (Read Input Registers)
    if (request[1] != 0x04) return;

    // Step 4: Validate CRC
    uint16_t crc_calc = modbus_crc(request, 6);
    uint16_t crc_rx   = request[6] | (request[7] << 8);
    if (crc_calc != crc_rx) {
        ESP_LOGW("modbus_rx", "CRC fout!");
        return;
    }

    // Step 5: Check P1 data validity
    // If P1 is unavailable, send no response at all
    // The Growatt will time out and stop producing — same as a real SDM630 failure
    if (!p1_valid()) {
        ESP_LOGW("modbus", "P1 niet beschikbaar, geen response verstuurd");
        return;
    }

    // Extract start register and count from request
    uint16_t start_reg = (request[2] << 8) | request[3];
    uint16_t reg_count = (request[4] << 8) | request[5];

    // Step 6: Get current grid data and build response
    GridData g = get_grid_data();

    uint8_t response[256];
    response[0] = 0x02;           // Slave address
    response[1] = 0x04;           // Function code FC04
    response[2] = reg_count * 2;  // Byte count = registers * 2

    int offset = 3;
    for (uint16_t r = 0; r < reg_count; r += 2) {
        // Get float value for this register
        float val = get_register_value(start_reg + r, g);

        // Pack IEEE 754 float32 into 4 bytes, big-endian word order
        uint32_t raw;
        memcpy(&raw, &val, sizeof(float));
        response[offset++] = (raw >> 24) & 0xFF;
        response[offset++] = (raw >> 16) & 0xFF;
        response[offset++] = (raw >>  8) & 0xFF;
        response[offset++] =  raw        & 0xFF;
    }

    // Append CRC16 (little-endian)
    uint16_t crc = modbus_crc(response, offset);
    response[offset++] = crc & 0xFF;
    response[offset++] = (crc >> 8) & 0xFF;

    // Step 7: Send response to Growatt
    for (int i = 0; i < offset; i++)
        id(modbus_bus).write_byte(response[i]);

    ESP_LOGI("modbus_tx", "Reg 0x%04X | L1=%.0fW L2=%.0fW L3=%.0fW | Totaal=%.0fW",
             start_reg, g.p[0], g.p[1], g.p[2],
             g.p[0] + g.p[1] + g.p[2]);

    // Flush any stray bytes that arrived during processing
    uint8_t dummy;
    while (id(modbus_bus).available() > 0)
        id(modbus_bus).read_byte(&dummy);
}


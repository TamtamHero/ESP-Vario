# pragma once
#include "math.h"
#include "stdbool.h"

#define DEG_TO_RAD 0.017453292519943295769236907684886
#define _180_DIV_PI         57.295779f

#define ABS(x) ((x) < 0 ? -(x) : (x))

#define CLAMP(x,mn,mx)       {if (x <= (mn)) x = (mn); else if (x >= (mx)) x = (mx);}

// Constants for barometric formula
#define SEA_LEVEL_PRESSURE 101325.0  // Pa (standard atmosphere)
#define TEMPERATURE_LAPSE_RATE 0.0065  // K/m (standard)
#define STANDARD_TEMP 288.15  // K (15°C at sea level)
#define G_ACCEL 9.80665  // m/s²
#define AIR_MOLAR_MASS 0.0289644  // kg/mol
#define UNI_GAS_CONST 8.31432  // N·m/(mol·K)

// input pressur in Pascals, altitude output in Meters
static inline float pressure_to_altitude(float pressure_pa, bool cm) {
    // Calculate the exponent denominator once
    const float exponent_denominator = (UNI_GAS_CONST * TEMPERATURE_LAPSE_RATE) /
                                       (G_ACCEL * AIR_MOLAR_MASS);

    // Barometric formula inverted to solve for altitude
    float altitude_m = (STANDARD_TEMP / TEMPERATURE_LAPSE_RATE) *
                       (1.0 - pow(pressure_pa / SEA_LEVEL_PRESSURE, exponent_denominator));

    // Convert meters to centimeters and round to nearest integer
    return cm ? altitude_m * 100 : altitude_m;;
}

// altitude input in Meters, pressure output in Pascals
static inline float altitude_to_pressure(float altitude_m, bool hpa) {
    // Barometric formula calculation
    float pressure = SEA_LEVEL_PRESSURE *
                     pow(1 - (TEMPERATURE_LAPSE_RATE * altitude_m) / STANDARD_TEMP,
                         (G_ACCEL * AIR_MOLAR_MASS) /
                         (UNI_GAS_CONST * TEMPERATURE_LAPSE_RATE));

    return hpa ? pressure / 100 : pressure;
}
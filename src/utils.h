#ifndef UTILS_H
#define UTILS_H

// Add Noise to a Signal
float AddNoise(float signal, float amplitude);

// Map Value from One Range to Another
float MapValue(float value, float in_min, float in_max, float out_min,
               float out_max);

// Clamp Value within a Range
float RClamp(float value, float min, float max);

#endif // UTILS_H

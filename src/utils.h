#ifndef UTILS_H
#define UTILS_H

// Define Maximum Data Points for Graphs
#define MAX_DATA_POINTS 1000

// Update History for Graphs
void UpdateGraph(float *history, int *index, float value);

// Map Value from One Range to Another
float MapValue(float value, float in_min, float in_max, float out_min,
               float out_max);

// Add Noise to a Signal
float AddNoise(float signal, float amplitude);

#endif // UTILS_H

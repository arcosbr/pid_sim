// log.h
#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <stdio.h>
#include <stdbool.h>

// Data Logger Structure
typedef struct
{
    FILE *file;
} DataLogger;

// Initialize Data Logger with a file name
bool DataLogger_Init(DataLogger *logger, const char *filename);

// Log Data to CSV
void DataLogger_LogData(DataLogger *logger, float time, float angle,
                        float error, float pressure);

// Reset Data Logger (close current file and open a new one)
bool DataLogger_Reset(DataLogger *logger, const char *filename);

// Close Data Logger
void DataLogger_Close(DataLogger *logger);

#endif // DATA_LOGGER_H

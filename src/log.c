// log.c
#include "log.h"

// Initialize Data Logger with a file name
bool DataLogger_Init(DataLogger *logger, const char *filename)
{
    logger->file = fopen(filename, "w");

    if (logger->file == NULL)
    {
        printf("Failed to open data log file: %s\n", filename);
        return false;
    }

    // Write CSV header
    fprintf(logger->file, "Time(s),Angle(rad),Error(rad),Pressure(bar)\n");
    return true;
}

// Log Data to CSV
void DataLogger_LogData(DataLogger *logger, float time, float angle,
                        float error, float pressure)
{
    if (logger->file != NULL)
    {
        fprintf(logger->file, "%.4f,%.4f,%.4f,%.4f\n", time, angle, error,
                pressure);
    }
}

// Reset Data Logger (close current file and open a new one)
bool DataLogger_Reset(DataLogger *logger, const char *filename)
{
    if (logger->file != NULL)
    {
        fclose(logger->file);
    }

    return DataLogger_Init(logger, filename);
}

// Close Data Logger
void DataLogger_Close(DataLogger *logger)
{
    if (logger->file != NULL)
    {
        fclose(logger->file);
        logger->file = NULL;
    }
}

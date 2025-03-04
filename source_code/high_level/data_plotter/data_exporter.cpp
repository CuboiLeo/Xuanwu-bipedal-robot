#include "data_exporter.h"

#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <iostream>

void logDataToCSV(double data1, double data2, double data3, double data4, double data5, const std::string& filename)
{
    // Get current time
    std::time_t now = std::time(nullptr);
    std::tm* localTime = std::localtime(&now);

    // Format the timestamp as YYYY-MM-DD HH:MM:SS
    char timeBuffer[20];
    std::strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", localTime);

    // Open CSV file in append mode
    std::ofstream outFile(filename, std::ios_base::app);
    if (!outFile.is_open())
    {
        std::cerr << "Error opening file " << filename << std::endl;
        return;
    }

    // Write timestamp and data values as a comma-separated line
    // If you only need the first two or three values, the extras will still be written 
    // with default zero values. Adjust as needed.
    outFile << timeBuffer << ","
            << data1 << ","
            << data2 << ","
            << data3 << ","
            << data4 << ","
            << data5
            << "\n";

    // Close the file
    outFile.close();
}

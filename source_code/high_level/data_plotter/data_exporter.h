#ifndef DATA_EXPORTER_H
#define DATA_EXPORTER_H

#include <string>

void logDataToCSV(
    double data1 = 0,
    double data2 = 0,
    double data3 = 0,
    double data4 = 0,
    const std::string &filename = "../data_plotter/log.csv"
);

#endif
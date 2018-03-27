#ifndef DATAREADER_H
#define DATAREADER_H

#include "allHeader.h"

class DataReader
{
public:
    DataReader();

     // function to get images names with path
    QStringList getImages(int type);

    //function to get raster data from dem file
    std::vector<double> getRaster(double &originX, double &originY, int &nCols, double &pWidth);

    //functions to split strings by delimiters
    std::vector<std::string> split(const std::string &s, char delim);
    template<typename Out>
    void split(const std::string &s, char delim, Out result);
};

#endif // DATAREADER_H

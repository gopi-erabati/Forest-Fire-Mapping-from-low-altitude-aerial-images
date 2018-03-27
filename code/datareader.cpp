#include "datareader.h"

DataReader::DataReader()
{
}

// function to get image
QStringList DataReader::getImages(int type){

    QStringList fileNames;
    if (type == 1){
        // read images from selection dialog
        fileNames = QFileDialog::getOpenFileNames(0, "Select images to segment", "/home/gerabati/project/firers_data/georef_Esperce_HauteGaronne/", "Image Files (*.png *.jpg *.bmp *.JPG)");
    }
    else{
        // read images from selection dialog
        fileNames = QFileDialog::getOpenFileNames(0, "Select images to segment", "/home/gerabati/project/firers_data/georef_Esperce_hauteGaronne_segmented/", "Image Files (*.png *.jpg *.bmp *.JPG)");
    }


    return fileNames;
}

std::vector<double> DataReader::getRaster(double &originX, double &originY, int &nCols, double &pWidth){

    std::vector<double> rasterDataDem;

    GDALDataset *gDataSet;
    double geoTransform[6];
    int nRows, nBands;
    double pHeight;

    //register the drivers
    GDALAllRegister();

    // get a DEM file
    gDataSet = (GDALDataset *) GDALOpen("/home/gerabati/project/firers_data/dem/esperce3_orthodsm_geo_40cm.tif", GA_ReadOnly);

    //check if data is present
    if (gDataSet == NULL){

        qDebug() << "No data found";

    }

    //get cols , rows and bands in file
    nCols = gDataSet->GetRasterXSize();
    nRows = gDataSet->GetRasterYSize();
    nBands = gDataSet->GetRasterCount();

    //get geotransform to get origin (X,Y) and pixel Width and Height
    gDataSet->GetGeoTransform(geoTransform);
    originX = geoTransform[0];
    originY = geoTransform[3];
    pWidth = geoTransform[1];
    pHeight = geoTransform[5];


    //buffer to read row
    float* dRow1 = (float*) CPLMalloc(sizeof(float)*nCols);

    //read row data and store in vector
    for(int i=0; i < nRows; ++i){

        // read a row
        gDataSet->GetRasterBand(1)->RasterIO(GF_Read, 0, i, nCols, 1, dRow1, nCols, 1, GDT_Float32, 0, 0);

        for (int j=0; j< nCols; ++j){

            rasterDataDem.push_back(dRow1[j]);
        }

    }

    CPLFree(dRow1);
    return rasterDataDem;

}

// functions to split strings
template<typename Out>
void DataReader::split(const std::string &s, char delim, Out result) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}

std::vector<std::string> DataReader::split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

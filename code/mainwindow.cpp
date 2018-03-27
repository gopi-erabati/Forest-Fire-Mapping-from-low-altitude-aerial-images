#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //projection.get3DWorldPoints2();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_uploadImagePush_clicked()
{
    //to get all filenames
    fileNames = datareader.getImages(1);

}

void MainWindow::on_segmentImgPush_clicked()
{
    //run for all files selected
    for(int i = 0; i < fileNames.size(); ++i){

        //get filename and read image and convert colorspace and resize it
        QString fileName = fileNames.at(i);
        cv::Mat inputImg = cv::imread(fileName.toStdString());
        cv::cvtColor(inputImg, inputImg, CV_BGR2RGB); // change bgr to rgb
        cv::resize(inputImg, inputImg, cv::Size(), 0.25, 0.25); // resize image

        //get the filename to print on nammed window
        std::string imgName = fileName.toStdString().substr(fileName.toStdString().length() - 12);
        cv::imshow(imgName, inputImg);

        //segment the image
        cv::Mat segmentedImg = segment.getSegmented(inputImg, ui->thresholdSpin->value());

        //path to save segemneted images
        char path[100] = "/home/gerabati/project/firers_data/georef_Esperce_hauteGaronne_segmented/";

        //to convert image name to char array
        char *imgNameC = new char[imgName.length() + 1];
        strcpy(imgNameC, imgName.c_str());

        //get fullpath to save file
        strcat(path, imgNameC);

        cv::imwrite(path, segmentedImg); //save segmented image
        strcat(imgNameC, "_segmented");
        cv::imshow(imgNameC, segmentedImg);

    }
}

void MainWindow::on_uploadImagePush_2_clicked()
{
    //to get all filenames
    fileNamesSeg = datareader.getImages(2);
}

void MainWindow::on_updateFireMapPush_clicked()
{

    //intrinsic parameters in pixels, scaled by 0.25 because, image is large and image is also scaled to 0.25!!!
    double intrinsicInPixels[] = {0.25*3272.1733924963492, 0, 0.25*2342.3086717022011, 0, 0.25*3272.1733924963492, 0.25*1770.4377498787001, 0, 0, 1};
    std::vector<double> intrinsicInPixelsVec(intrinsicInPixels, intrinsicInPixels+9);

    //tangential and radial distortion coeff
    double distCoeff[] = {-0.04646865617107581, 0.051288490946210602, -0.025988438162638149, 0.0032416606187316522, 0.0033995207337653736};
    std::vector<double> distCoeffVec(distCoeff, distCoeff+5);

    //to get raster data from dem file in vector and also metadata
    double originX, originY, pixelSize;
    int nCols;
    std::vector<double> rasterData = datareader.getRaster(originX, originY, nCols, pixelSize);
    double metaData[] = {originX, originY, nCols, pixelSize};
    std::vector<double> metaDataVec(metaData, metaData+4);

    //parametrised constructor of Projection class
    Projection projection1(intrinsicInPixelsVec, 0.000001339453, distCoeffVec, rasterData, metaDataVec, 0.0);

    //for reading camera parameters from txt file
    std::string line;
    std::string delimitter = " ";
    std::ifstream file;
    file.open("/home/gerabati/project/firers_data/georef_Esperce_HauteGaronne/files/flightData1.txt");

    //run for all files selected
    for (int i = 0; i < fileNamesSeg.size(); ++i){
        qDebug() << " projecting..." << (i+1);
        //get filename and read image
        QString fileName = fileNamesSeg.at(i);
        cv::Mat inputImg = cv::imread(fileName.toStdString());
        cv::resize(inputImg, inputImg, cv::Size(), 0.25, 0.25); // resize image if its large to handle

        //get the filename to get the camera parameters
        std::string imgName = fileName.toStdString().substr(fileName.toStdString().length() - 12);
        //to convert image name to char array
        char *imgNameC = new char[imgName.length() + 1];
        strcpy(imgNameC, imgName.c_str());

        //get camera parametes for this image from flightdata file
        if(file.is_open()){
            while(!file.eof()){
                std::getline(file, line);

                //split line to get imagename at first token with space demilimter
                std::string imgNameF = line.substr(0, line.find(delimitter));
                //to convert image name in file to char array
                char *imgNameFC = new char[imgNameF.length() + 1];
                strcpy(imgNameFC, imgNameF.c_str());

                if(strcmp(imgNameC, imgNameFC ) == 0){
                    std::vector<std::string> tempVal;
                    //get instrinsic parameters in pixels (3 rows of matrix)
                    for( int row = 0; row < 3; ++row){
                        std::getline(file, line);
                        std::vector<std::string> tempV = datareader.split(line, ' ');
                        tempVal.push_back(tempV.at(0)); tempVal.push_back(tempV.at(1)); tempVal.push_back(tempV.at(2));
                    }
                    //put intrinsic parameters vector into vector of double
                    std::vector<double> intrinsicsInPixels;
                    for (int z = 0; z < tempVal.size(); ++z){
                        intrinsicsInPixels.push_back(std::strtod(tempVal.at(z).c_str(), NULL));
                    }
                    tempVal.clear(); //clear temporary vector stack

                    //get distortion parameters
                    for (int row = 0; row < 2; ++row){
                        std::getline(file, line);
                        std::vector<std::string> tempV = datareader.split(line, ' ');
                        tempVal.push_back(tempV.at(0)); ; tempVal.push_back(tempV.at(1)); if (row == 0) {tempVal.push_back(tempV.at(2));};
                    }
                    //put distortio coeff vector into vector of double
                    std::vector<double> distCoeff;
                    for (int z = 0; z < tempVal.size(); ++z){
                        distCoeff.push_back(std::strtod(tempVal.at(z).c_str(), NULL));
                    }
                    tempVal.clear(); //clear temporary vector stack

                    //get trasnlation vector
                    std::getline(file, line);
                    tempVal = datareader.split(line, ' ');
                    //put translation vector into vector of double
                    std::vector<double> translationInMeters;
                    for (int z = 0; z < tempVal.size(); ++z){
                        translationInMeters.push_back(std::strtod(tempVal.at(z).c_str(), NULL));
                    }
                    tempVal.clear(); //clear temporary vector stack

                    //get roratation matrix
                    for( int row = 0; row < 3; ++row){
                        std::getline(file, line);
                        std::vector<std::string> tempVN = datareader.split(line, ' ');
                        tempVal.push_back(tempVN.at(0)); tempVal.push_back(tempVN.at(1)); tempVal.push_back(tempVN.at(2));
                    }
                    //put rotation vector into vector of double
                    std::vector<double> rotation;
                    for (int z = 0; z < tempVal.size(); ++z){
                        rotation.push_back(std::strtod(tempVal.at(z).c_str(), NULL));
                    }
                    tempVal.clear(); //clear temporary vector stack

                    cv::Mat fireMapDelta = projection1.projectAndUpdateFireMap(translationInMeters, rotation, inputImg);
                    //qDebug() << std::strtod(tempV.at(0).c_str(), NULL) <<  std::strtod(tempV.at(1).c_str(), NULL);


                    //save results
                    //path to save firemaps
                    char path[100] = "/home/gerabati/project/firers_data/";

                    //get fullpath to save file
                    std::string fireMapNum = std::to_string((i+1));
                    char* fireMapNumC = new char[10];
                    strcpy(fireMapNumC, "Ind");
                    strcat(fireMapNumC, fireMapNum.c_str());
                    strcat(fireMapNumC, ".JPG");
                    strcat(path, (const char*)fireMapNumC);
                    //cv::resize(fireMapDelta, fireMapDelta, cv::Size(), 0.4, 0.4);
                    cv::imwrite(path, fireMapDelta); //save segmented image

                    //cv::resize(fireMapDelta, fireMapDelta, cv::Size(), 0.25, 0.25);
                    //cv::imshow("image1", fireMapDelta);
                    cv::Mat fireMap = projection1.getFireMap();
                    //cv::imshow("image2", fireMap);

                    char path1[100] = "/home/gerabati/project/firers_data/";
                    //get fullpath to save file
                    std::string fireMapNum1 = std::to_string((i+1));
                    char* fireMapNumC1 = new char[fireMapNum1.length()+1];
                    strcpy(fireMapNumC1, fireMapNum1.c_str());
                    strcat(fireMapNumC1, ".JPG");
                    strcat(path1, (const char*)fireMapNumC1);

                    //cv::resize(fireMap, fireMap, cv::Size(), 0.4, 0.4);
                    cv::imwrite(path1, fireMap); //save firemap image

                    delete[] fireMapNumC;
                    delete[] fireMapNumC1;
                    delete [] imgNameC;
                    delete[] imgNameFC;
                    break; //break while loop to go to next image

                } // end of if condition
            }// end of while loop
        }//end of if condition to check file is open
        qDebug() << (i+1);
    } //end of for loop for images

}// end of function



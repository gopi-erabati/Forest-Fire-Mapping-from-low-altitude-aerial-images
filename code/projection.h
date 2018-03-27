#ifndef PROJECTION_H
#define PROJECTION_H

#include "allHeader.h"
#include "segment.h"

class Projection
{

    //constructor initialised variables
    std::vector<double> mcameraIntrinsicInPixels, mDistortionCoeff, mRasterDem, mMetaDataDem;
    double mPixelDensity, mThresholdSegment;
    cv::Mat mFireMap, mcameraIntrinsicInPixelsMat, mdistortionCoeffMat, mcameraIntrinsicInMetersMat; //create firemap with same size as DEM when object is called
    int mNoCols, mNoRows;


public:
    //external use functions

    //constructor to initalise parameters which dont change for every image like cameraintrinsics, pixeldensity, distcoeff, Dem data
    //cameraIntrinsicInPixels - intrinsic matrix of camera (K) (3x3) , which includes focal length in pxels and principal point - 9element vector
    //pixelDensity - scale factor to convert pixels to world units (meters) here = 0.00000134 m/pixel - one double value
    //distortionCoeff - tangential(P1,P2,P3) and radial(K1,K2) distortion coeff of camera lens - 5element vector
    //rasterDem - elevation data of dem
    //metaDataDem - originX, originY, Ncols, pixelSize of dem- 4element vector
    //thresholdSegment - threshold value to segment image (0-255)
    Projection(std::vector<double> cameraIntrinsicInPixels, double pixelDensity, std::vector<double> distortionCoeff, std::vector<double> rasterDem, std::vector<double> metaDataDem, double thresholdSegment);

    //function to map 3d points to 2dpoints using camera matrix
    // position - the x,y,z position of image capture (meters) - 3element vector
    //rotation - the rotation matrix of camera (3x3) - 9element vector (if you have pitch, roll, yaw - convert them to rotation matrix)
    //inputImg - image to be projected onto dem in cv::mat format
    //output - projected image on dem in cv::mat format (its not total updated projections, just delta)
    cv::Mat projectAndUpdateFireMap(std::vector<double> position, std::vector<double> rotation, cv::Mat inputImg);


    //function to get full updated firemap
    //output - overall projections of images on the dem
    cv::Mat getFireMap();


private:
    //functions used internally by program

    //function to undistort the distorted images due to camera Radial and Tangential distortion
    cv::Mat getUndistortedImage(cv::Mat inputImg, cv::Mat cameraMatrix, cv::Mat distMatrix);

    //function to calculate camera matrix given intrinsic matrix, rotation and translation matrix
    cv::Mat calculateCameraMatrix(cv::Mat intrinsicsInMeters, cv::Mat rotation, cv::Mat translationInMeters);



};

#endif // PROJECTION_H

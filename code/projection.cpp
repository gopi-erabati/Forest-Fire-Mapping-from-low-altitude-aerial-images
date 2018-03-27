#include "projection.h"

//constructor definition
Projection::Projection(std::vector<double> cameraIntrinsicInPixels, double pixelDensity, std::vector<double> distortionCoeff, std::vector<double> rasterDem, std::vector<double> metaDataDem, double thresholdSegment):
    mcameraIntrinsicInPixels(cameraIntrinsicInPixels), mPixelDensity(pixelDensity), mDistortionCoeff(distortionCoeff), mRasterDem(rasterDem), mMetaDataDem(metaDataDem), mThresholdSegment(thresholdSegment)
{
    //rows = (rastersize)/(no of cols)
    mNoCols = (int)metaDataDem.at(2);
    mNoRows = rasterDem.size()/mNoCols;

    mFireMap = cv::Mat::zeros(mNoRows, mNoCols, CV_8UC1); //create firemap to update

    // camera parameters
    mcameraIntrinsicInPixelsMat = (cv::Mat_<double>(3,3) << mcameraIntrinsicInPixels.at(0), mcameraIntrinsicInPixels.at(1), mcameraIntrinsicInPixels.at(2), mcameraIntrinsicInPixels.at(3), mcameraIntrinsicInPixels.at(4), mcameraIntrinsicInPixels.at(5), mcameraIntrinsicInPixels.at(6), mcameraIntrinsicInPixels.at(7), mcameraIntrinsicInPixels.at(8));
    mcameraIntrinsicInMetersMat = pixelDensity * mcameraIntrinsicInPixelsMat;
    mdistortionCoeffMat = (cv::Mat_<double>(1,5) << mDistortionCoeff.at(0), mDistortionCoeff.at(1), mDistortionCoeff.at(3), mDistortionCoeff.at(4), mDistortionCoeff.at(2));
}

//function to undistort the distorted images due to camera Radial and Tangential distortion
cv::Mat Projection::getUndistortedImage(cv::Mat inputImg, cv::Mat cameraMatrix, cv::Mat distMatrix){

    //resize the large image if required then we need to change camera matrix by same order
    cv::Mat undistortedImg; // for undistortion of images
    cv::undistort(inputImg, undistortedImg, cameraMatrix, distMatrix);

    return undistortedImg;

}


//function to calculate camera matrix given intrinsic matrix, rotation and translation matrix
cv::Mat Projection::calculateCameraMatrix(cv::Mat intrinsicsInMeters, cv::Mat rotation, cv::Mat translationInMeters){

    cv::Mat extrinsics,affineRow, zeroCol, cameraMatrix;
    //P = K [R|-Rt]
    // [R|-Rt] extrinsics
    cv::hconcat(rotation, -1* rotation * translationInMeters, extrinsics);
    affineRow = (cv::Mat_<double>(1,4)<<0.0, 0.0, 0.0, 1.0);
    cv::vconcat(extrinsics, affineRow, extrinsics );

    //K Intrinsics
    zeroCol = (cv::Mat_<double>(3,1)<<0.0, 0.0, 0.0);
    cv::hconcat(intrinsicsInMeters, zeroCol, intrinsicsInMeters);

    cameraMatrix  = intrinsicsInMeters * extrinsics;

    return cameraMatrix;

}


//function to map 3d points to 2dpoints using camera matrix
cv::Mat Projection::projectAndUpdateFireMap(std::vector<double> position, std::vector<double> rotation, cv::Mat inputImg){

    double originX = mMetaDataDem.at(0);
    double originY = mMetaDataDem.at(1);
    double pixelSize = mMetaDataDem.at(3);

    //find pixel cordinates where the position of image capture is located
    int xOffset = (int)(position.at(0) - originX)/pixelSize;
    int yOffset = (int)(position.at(1) - originY)/(-1*pixelSize);


    //pan around +/- 2*505.492 (altitude) around this point
    // which is around 1000m , so find the number of pixels to go back to top left corner
    int nGridsX = (int)1000/pixelSize;
    int nGridsY = (int)1000/pixelSize;
    xOffset = xOffset - nGridsX; yOffset = yOffset - nGridsY;

    //create a local firemap to show projection of each image
    cv::Mat fireMapDelta = cv::Mat::zeros(mNoRows, mNoCols, CV_8UC1); //create firemap to update

    // openCV reads images as BGR , so convert to RGB
    cv::cvtColor(inputImg, inputImg, CV_BGR2RGB); // change bgr to rgb

    //undistort the input image
    cv::Mat undistortedImg = getUndistortedImage(inputImg, mcameraIntrinsicInPixelsMat, mdistortionCoeffMat );

    //get camera matrix
    cv::Mat rotationMat = (cv::Mat_<double>(3,3) << rotation.at(0), rotation.at(1), rotation.at(2), rotation.at(3), rotation.at(4), rotation.at(5), rotation.at(6), rotation.at(7), rotation.at(8));
    cv::Mat translationMat = (cv::Mat_<double>(3,1) << position.at(0), position.at(1), position.at(2));
    cv::Mat cameraMatrix = calculateCameraMatrix(mcameraIntrinsicInMetersMat, rotationMat, translationMat );

    //SEGMENTATAION of Image // see Segment class to choose method to segment
    //1. Normal one channel (R) thresholding (good)
    //2. Conversion from RGB2HSI and thresholding
    //3. Iterative thresholding (bad)
    // Segmentation class object
    //    Segment segment;
    //    undistortedImg =  segment.getSegmented(undistortedImg, mThresholdSegment);

    //change color image to gray
    if (inputImg.channels() == 3){
        cv::cvtColor(undistortedImg, undistortedImg, CV_RGB2GRAY); // change bgr to rgb
    }


    //now project the points in 2*nGridsx and 2*nGridsY area to get 2Dpoints
    for(int i = 0; i < mNoRows-1; ++i){ //row

        //check if i is out of dem image
        if (i < 0 || i >= mNoRows){
            continue;
        }

        for(int j = 0; j < mNoCols-1; ++j){ //col

            //check if j is out of dem image
            if (j < 0 || j >= mNoCols){
                continue;
            }


            std::vector<cv::Point3d> points3D;
            cv::Point3d point1, point2, point3, point4;
            // get X, Y Z of four points to project onto image
            point1.x = originX + j*pixelSize +pixelSize/2.0f ; point1.y = originY - i*pixelSize - pixelSize/2.0f ; point1.z = mRasterDem.at(mNoCols*i + j);
            point2.x = point1.x + pixelSize; point2.y = point1.y; point2.z = mRasterDem.at(mNoCols*i + j + 1);
            point3.x = point1.x; point3.y = point1.y - pixelSize; point3.z = mRasterDem.at(mNoCols*(i+1) + j);
            point4.x = point2.x; point4.y = point3.y; point4.z = mRasterDem.at(mNoCols*(i+1) + j + 1);


            //push the four points to the vector
            points3D.push_back(point1);
            points3D.push_back(point2);
            points3D.push_back(point3);
            points3D.push_back(point4);

            std::vector<cv::Mat> p12D; //vector to store 2D points
            p12D.reserve(4);

            bool flag = false;

            //project four 3D points to get 2D points on image plane
            for (int k = 0; k < 4; k++){
                cv::Mat p1 = (cv::Mat_<double>(4,1)<<points3D[k].x, points3D[k].y, points3D[k].z, 1.0);
                p1 = cameraMatrix * p1;
                p1.at<double>(0,0) = (int) (p1.at<double>(0,0) /(p1.at<double>(2,0))); // divide by Z-cordinate
                p1.at<double>(1,0) = (int) (p1.at<double>(1,0) /(p1.at<double>(2,0))); // divide by Z-cordinate
                p12D.push_back(p1.clone());
            }

            //check whether points are on image plane
            if ((p12D[0].at<double>(0,0) < 0 || p12D[0].at<double>(0,0) > inputImg.cols || p12D[0].at<double>(1,0) < 0 || p12D[0].at<double>(1,0) > inputImg.rows) &&
                    (p12D[1].at<double>(0,0) < 0 || p12D[1].at<double>(0,0) > inputImg.cols || p12D[1].at<double>(1,0) < 0 || p12D[1].at<double>(1,0) > inputImg.rows) &&
                    (p12D[2].at<double>(0,0) < 0 || p12D[2].at<double>(0,0) > inputImg.cols || p12D[2].at<double>(1,0) < 0 || p12D[2].at<double>(1,0) > inputImg.rows) &&
                    (p12D[3].at<double>(0,0) < 0 || p12D[3].at<double>(0,0) > inputImg.cols || p12D[3].at<double>(1,0) < 0 || p12D[3].at<double>(1,0) > inputImg.rows)){
                flag = true;
            }

            //if no data dont continue remaining steps
            if (flag){

                continue;
            }

            //create  a poly mask with this 2D points to get the instensity values in that mask of original image
            cv::Mat maskImg = cv::Mat::zeros(inputImg.rows, inputImg.cols, CV_8UC1);
            cv::Point points2D[1][4];
            for (int l = 0; l < 4 ; l++){
                points2D[0][l] = cv::Point(p12D[l].at<double>(0,0), p12D[l].at<double>(1,0));
            }

            const cv::Point* ppt[1] = { points2D[0] };
            int npt[] = { 4 };
            cv::fillPoly( maskImg, ppt, npt, 1, cv::Scalar(255));

            //get the average intensity of area defined by polygon mask
            cv::Scalar meanIntensityS = cv::mean(undistortedImg, maskImg);
            double meanIntensity = meanIntensityS.val[0];

            // uncomment the if condition if using eith fire segmented images or comment if using to project normal images
            //check if mean intensity is greater than 0.5 and paint the DEM pixel accordingly
            //if (meanIntensity > 0.5){
            mFireMap.at<uchar>(i,j) = meanIntensity;
            fireMapDelta.at<uchar>(i,j) = meanIntensity;
            //}

        }
    }

    //    cv::resize(fireMapDelta, fireMapDelta, cv::Size(), 0.4, 0.4);
    return fireMapDelta;

}

//function to get full updated firemap
cv::Mat Projection::getFireMap(){

    return mFireMap;
}

#ifndef SEGMENT_H
#define SEGMENT_H


#include "allHeader.h"

class Segment
{

public:
    // function to threshold image on one channel (R-channel for RGB images)
    cv::Mat getSegmented(cv::Mat inputImg, double thresholdValue);

    //get segmented image from conversion of HSI colorpsca and segmenting HUE
    cv::Mat getSegmentedROI(cv::Mat inputImg, double thresholdValue);

    //get segment image by iterative thresholding
    cv::Mat getSegmentedIterativeThresholding(cv::Mat inputImg);

private:
    //function to normalise RGB image
    cv::Mat normaliseRGB(cv::Mat inputImg);

    //to convert BGR to HSI
    cv::Mat getBGR2HSI(cv::Mat inputImg);

};

#endif // SEGMENT_H

#include "segment.h"
#include <typeinfo>


// function to get segmnetd image basing on thresholding
cv::Mat Segment::getSegmented(cv::Mat inputImg, double thresholdValue){

    cv::Mat outputImg;
    //check if image is binary
    if (inputImg.channels() == 1){
        cv::threshold(inputImg, outputImg, thresholdValue, 255, cv::THRESH_BINARY);
    }
    else{
        //change color spaces from BGR to RGB
        cv::cvtColor(inputImg, inputImg, CV_BGR2RGB);

        std::vector<cv::Mat> img_splited;
        // split the RGB image
        cv::split(inputImg, img_splited);

        //apply threshold on ech channel
        for (size_t i=0; i < img_splited.size(); i++){
            cv::threshold(img_splited[i], img_splited[i], thresholdValue, 255, cv::THRESH_BINARY);
        }

        outputImg = img_splited[0];

        //        //for mask with thresholded channels
        //        thresholded = img_splited[0].clone();
        //        for (size_t i = 0; i < img_splited.size(); i++){
        //            thresholded &= img_splited[i];
        //        }

        //        // apply mask on input image
        //        outputImg = inputImg.clone();
        //        outputImg &= thresholded;

    }

    return outputImg;
}

//function to normalise RGB image
cv::Mat Segment::normaliseRGB(cv::Mat inputImg){

    // data vraibles for normaliseRGB()
    cv::Mat rgbFrames[3];
    cv::Mat inputImgNorm; // for rgb normalisation

    // Extract the color planes and calculate I = (r + g + b) / 3
    cv::split(inputImg, rgbFrames);

    cv::Mat intensity_f((rgbFrames[0] + rgbFrames[1] + rgbFrames[2]) / 3.0f);

    cv::divide(rgbFrames[0], intensity_f, rgbFrames[0]);
    cv::divide(rgbFrames[1], intensity_f, rgbFrames[1]);
    cv::divide(rgbFrames[2], intensity_f, rgbFrames[2]);

    rgbFrames[0].convertTo(rgbFrames[0], CV_8UC1, 255.0);
    rgbFrames[1].convertTo(rgbFrames[1], CV_8UC1, 255.0);
    rgbFrames[2].convertTo(rgbFrames[2], CV_8UC1, 255.0);

    cv::merge(rgbFrames, 3, inputImgNorm);

    return inputImgNorm;
}

//to convert BGR to HSI
cv::Mat Segment::getBGR2HSI(cv::Mat inputImg){

    cv::Mat hsi(inputImg.rows, inputImg.cols, inputImg.type());
    for(int i = 0; i < inputImg.rows; i++)
    {
        for(int j = 0; j < inputImg.cols; j++)
        {
            //data variables for getBGR2HSI()
            float r, g, b, h, s, in; // to store rgb and hsi values
            int min_val; // to store min value for rgb2hsi conversion

            // get BGR values for each pixel
            b = inputImg.at<cv::Vec3f>(i, j)[0];
            g = inputImg.at<cv::Vec3f>(i, j)[1];
            r = inputImg.at<cv::Vec3f>(i, j)[2];

            in = (b + g + r) / 3; // I value

            // get S value
            min_val = std::min(r, std::min(b,g));
            s = 1 - 3*(min_val/(b + g + r));
            if(s < 0.00001)
            {
                s = 0;
            }else if(s > 0.99999){
                s = 1;
            }

            //get H value
            if(s != 0)
            {
                h = 0.5 * ((r - g) + (r - b)) / sqrt(((r - g)*(r - g)) + ((r - b)*(g - b)));
                h = acos(h);

                if(b <= g)
                {
                    h = h;
                } else{
                    h = ((360 * 3.14159265) / 180.0) - h;
                }
            }
            //h = h/(2*3.14159265);

            hsi.at<cv::Vec3f>(i, j)[0] = h;
            hsi.at<cv::Vec3f>(i, j)[1] = s;
            hsi.at<cv::Vec3f>(i, j)[2] = in;
        }
    }

    return hsi;
}


//get segmented image from above two function
cv::Mat Segment::getSegmentedROI(cv::Mat inputImg, double thresholdValue){

    cv::Mat outputImg;

    //check if image is binary
    if (inputImg.channels() == 1){
        thresholdValue *= 255.0;
        cv::threshold(inputImg, outputImg, thresholdValue, 255, cv::THRESH_BINARY);
    }
    else{

        // data variables for getSegmnetedROI()
         cv::Mat hsiImg; // to store HSI and sobel images of input image
         cv::Mat hsiFrames[3];
         cv::Mat binaryMaskSaturation;

        //convert to floating point
        inputImg.convertTo(inputImg, CV_32FC3);

        //get normalised image
        cv::Mat inputImgNorm = normaliseRGB(inputImg);

        inputImgNorm.convertTo(inputImgNorm, CV_32FC3);

        // convert BGR2HSI using defined function above
        hsiImg = getBGR2HSI(inputImg);


        //split hsi image
        cv::split(hsiImg, hsiFrames);

        //cv::imshow("h", hsiFrames[0]);
        //cv::imshow("s", hsiFrames[1]);

        //check for saturation and produce binary mask
        cv::threshold(hsiFrames[1], binaryMaskSaturation, thresholdValue, 1.0, CV_THRESH_BINARY);

        //cv::imshow("mask", binaryMaskSaturation);

        cv::multiply(hsiFrames[0], binaryMaskSaturation, hsiFrames[0]);
        //cv::imshow("h after mask", hsiFrames[0]);

        //threshold hue and sat product image to get segmented image
        cv::threshold(hsiFrames[0], outputImg, 0.2, 255, CV_THRESH_BINARY);

    }
    return outputImg;
}


//get segment image by iterative thresholding
cv::Mat Segment::getSegmentedIterativeThresholding(cv::Mat inputImg){

    // data variables for getSegmentedIterativeThresholding()
    cv::Mat switchFun;
    long long int backgroundCount, backgroundSum, backgroundAvg;
    long long int objectCount, objectSum, objectAvg, thresholdVal;

    //convert input image to gray if its color image
    if (inputImg.channels() == 3){
        cv::cvtColor(inputImg, inputImg, CV_BGR2GRAY);
    }

    //create initial switching function (image - binary)
    switchFun = cv::Mat::ones(inputImg.rows, inputImg.cols, CV_8UC1);
    cv::multiply(switchFun, cv::Scalar::all(255), switchFun);

    //create background around four corners of initail switch function
    //top left
    for (int i = 0; i < 0.2*inputImg.rows ; i++){
        for (int j = 0; j < 0.2*inputImg.cols; j++){
            switchFun.at<uchar>(i,j) = 0;
        }
    }
    //top right
    for(int i = 0; i <= 0.2*inputImg.rows; i++ ){
        for(int j = 0.8*inputImg.cols; j < inputImg.cols; j++){
            switchFun.at<uchar>(i,j) = 0;
        }
    }
    //bottom left
    for(int i = 0.8*inputImg.rows; i < inputImg.rows; i++){
        for(int j = 0; j < 0.2*inputImg.cols; j++){
            switchFun.at<uchar>(i,j) = 0;
        }
    }
    //bottom right
    for (int i = 0.8*inputImg.rows; i < inputImg.rows; i++){
        for (int j = 0.8*inputImg.cols; j < inputImg.cols; j++){
            switchFun.at<uchar>(i,j) = 0;
        }
    }
    cv::imshow("switch fun ini", switchFun);
    for (int numTimes = 1; numTimes < 7; numTimes++){
        backgroundCount = 0;
        objectCount = 0;
        backgroundSum = 0;
        objectSum = 0;
        //basing on switch function assign the pixels to background or object integrator
        for(int i = 0; i < switchFun.rows; i++){
            for(int j = 0; j< switchFun.cols; j++){

                //for background integrator
                if (switchFun.at<uchar>(i,j) == 0){
                    backgroundSum += inputImg.at<uchar>(i,j);
                    backgroundCount += 1;
                }
                else{
                    objectSum += inputImg.at<uchar>(i,j);
                    objectCount += 1;
                }

            }
        }

        //average the background and object sums
        backgroundAvg = backgroundSum / backgroundCount;
        objectAvg = objectSum / objectCount;

        // get the background and object avg as new threshold
        thresholdVal = (backgroundAvg + objectAvg) / 2;

        cv::threshold(inputImg, switchFun, thresholdVal, 255, CV_THRESH_BINARY);

        //cv::imshow(boost::lexical_cast<std::string>(numTimes), switchFun);


    }
    return switchFun;

}


































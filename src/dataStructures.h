//
//  dataStructures.h
//  OpenCVTest
//
//  Created by Andreas Haja on 01.04.19.
//  Copyright © 2019 Andreas Haja. All rights reserved.
//

#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <opencv2/core.hpp>


struct DataFrame { // represents the available sensor information at the same time instance
    
    cv::Mat cameraImg; // camera image
    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
};


#endif /* dataStructures_h */

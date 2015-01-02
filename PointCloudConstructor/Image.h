/*
 * Image.h
 *
 *  Created on: Dec 9, 2014
 *      Author: ailinca
 */

#include <cv.h>
#include <highgui.h>
#include <memory>

#ifndef IMAGE_H_
#define IMAGE_H_

class Image {
	public:
    Image(std::string folder, 
        int imgID, 
        std::shared_ptr<cv::FeatureDetector> fdetector);
    std::vector<cv::KeyPoint> getFeatures();

  private:
		cv::Mat imgMat;
    cv::Point3f cameraPose;
    std::vector<cv::KeyPoint> features;

};


#endif /* IMAGE_H_ */

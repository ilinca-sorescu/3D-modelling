/*
 * PointCloudConstructor.h
 *
 *  Created on: Dec 8, 2014
 *      Author: ailinca
 */

#ifndef POINTCLOUDCONSTRUCTOR_H_
#define POINTCLOUDCONSTRUCTOR_H_

#include "Image.h"
#include <memory>
#include <cv.h>

class PointCloudConstructor {
	public:
		PointCloudConstructor(std::string folder);
    std::vector<cv::Point3f> getPoints();

	private:
		std::vector<std::shared_ptr<Image>> images;
    std::shared_ptr<cv::FeatureDetector> featureDetector;
};


#endif /* POINTCLOUDCONSTRUCTOR_H_ */

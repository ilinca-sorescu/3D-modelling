/*
 * PointCloudConstructor.h
 *
 *  Created on: Dec 8, 2014
 *      Author: ailinca
 */

#ifndef POINTCLOUDCONSTRUCTOR_H_
#define POINTCLOUDCONSTRUCTOR_H_

#define K_ 1

#include "Image.h"
#include "FeatureMatcher.h"
#include <memory>
#include <cv.h>
#include <numeric>

class PointCloudConstructor {
  public:
		PointCloudConstructor(std::string folder);
    std::vector<cv::Point3d> getPoints();
   	std::vector<std::shared_ptr<Image>> getImages();
    Point3d triangulate(Matx34d, Matx34d, Point2d, Point2d);

  private:
		std::vector<std::shared_ptr<Image>> images;
    std::unique_ptr<FeatureMatcher> featureMatcher;
    std::vector<std::vector<int>> kclosest;
    std::vector<std::vector<std::vector<cv::DMatch>>> matches;
    void compute_kclosest();
};

#endif /* POINTCLOUDCONSTRUCTOR_H_ */

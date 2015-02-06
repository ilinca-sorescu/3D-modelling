/*
 * PointCloudConstructor.h
 *
 *  Created on: Dec 8, 2014
 *      Author: ailinca
 */

#ifndef POINTCLOUDCONSTRUCTOR_H_
#define POINTCLOUDCONSTRUCTOR_H_

#define K_ 5

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
    Point3d triangulate(Matx44d, Matx44d, Point3d, Point3d);

  private:
		std::vector<std::shared_ptr<Image>> images;
    std::unique_ptr<FeatureMatcher> featureMatcher;
    std::vector<std::vector<int>> kclosest;
    std::vector<std::vector<std::vector<cv::DMatch>>> matches;
    void compute_kclosest();

    //p1 - the 2D point(in world coordinates) corresponding to the
    //     location of the feature with id m.trainIdx in img1.
    //p2 - the 2D point(in world corrdinates) corresponding to the
    //     location of the feature with id m.queryIdx in img2.
    void imageCoordinatesOfDMatch(
        DMatch m,
        shared_ptr<Image> img1,
        shared_ptr<Image> img2,
        Point2d& p1,
        Point2d& p2);
};

#endif /* POINTCLOUDCONSTRUCTOR_H_ */

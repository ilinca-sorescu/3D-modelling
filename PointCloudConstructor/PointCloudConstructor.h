/*
 * PointCloudConstructor.h
 *
 *  Created on: Dec 8, 2014
 *      Author: ailinca
 */

#ifndef POINTCLOUDCONSTRUCTOR_H_
#define POINTCLOUDCONSTRUCTOR_H_

#define K_ 1//10

#include "Image.h"
#include "FeatureMatcher.h"
#include <memory>
#include <cv.h>
#include <numeric>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

class PointCloudConstructor {
  public:
    static double MinRatio;
    static double MaxRatio;
    static double ReprojectionError;
    static double Tolerance;
    PointCloudConstructor(
        std::string folder,
        int wantedNumberOfPics = -1);
   	std::vector<std::shared_ptr<Image>> getImages();
    Point3d triangulate(Matx34d, Matx34d, Point2d, Point2d, int, int);
    Point3d ReadjustedTriangulation(Matx34d, Matx34d, Point2d, Point2d, int, int);
    void populateCloud(std::vector<cv::Point3d>);
    void cloudToPCD(String PCDFileName); //Point Cloud Data (out) file
    void cloudToTxt(String txtFileName);
    void normalizeCloudValues(); //make all point coords. in [-1, 1]
    std::vector<cv::Point3d> sortPoints(std::vector<cv::Point3d>);

  private:
    struct TriangulatedPoint{
      Point3d point;
      double reprojectionError;
      int imgId1;
      int imgId2;
      int featureId1;
      int featureId2;
    };

    //for each picture search for matches in its k closest neighbours
    std::vector<std::pair<cv::Point3d, double>> computePoints_kclosest();

    //search for matches in all of the pairs of images (img1, img2) where
    //the ratio between the distance between cameras 1 and 2 and the radius
    //of the viewing sphere is in the specified interval
    std::vector<std::pair<cv::Point3d, double>> computePoints_ratioInterval();

    std::vector<cv::Point3d> computePoints();

    std::vector<TriangulatedPoint> computePointsBetweenTwoImgs(int, int);

    std::vector<std::shared_ptr<Image>> images;
    std::unique_ptr<FeatureMatcher> featureMatcher;
    std::vector<std::vector<int>> kclosest;
    std::vector<std::vector<std::vector<cv::DMatch>>> matches;
    void compute_kclosest();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    std::vector<cv::Point3d> filterByReprojectionError(
        std::vector<std::pair<cv::Point3d, double>>);

    //computes error in equ. x = CameraMat * X
    double reprojectionError(
        cv::Matx34d,
        cv::Point2d,
        cv::Point3d);
};

#endif /* POINTCLOUDCONSTRUCTOR_H_ */

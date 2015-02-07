#ifndef FEATUREMATCHER_H_
#define FEATUREMATCHER_H_

#include "Image.h"
#include <memory>

class FeatureMatcher {
  public:
    FeatureMatcher(std::vector<std::shared_ptr<Image>>);

    std::vector<cv::DMatch> match(
        unsigned int,
        unsigned int,
        bool draw=false);

    //p1 - the 2D point(in camera coordinates) corresponding to the
    //     location of the feature with id m.queryIdx in img1.
    //p2 - the 2D point(in camera corrdinates) corresponding to the
    //     location of the feature with id m.trainIdx in img2.
    void static cameraCoordinatesOfDMatch(
        DMatch m,
        shared_ptr<Image> img1,
        shared_ptr<Image> img2,
        Point2d& p1,
        Point2d& p2);

  private:
    std::vector<std::shared_ptr<Image>> images;

    FlannBasedMatcher matcher;

    /*
     * Returns only the matches which make epipolar sense.
     * PRECONDITION: all of the matches have the same
     *  queryIdx and the same trainIdx.
     * img1 - image containing queryIdx.
     * img2 - image containing trainIdx.
     */
    std::vector<cv::DMatch> filterMatches(
        std::vector<cv::DMatch> matches,
        std::shared_ptr<Image> img1,
        std::shared_ptr<Image> img2);

    /*
     * The precision used to decide if a point
     * belongs to a line.
     */
    const double tolerance = 0.01;

    /*
     * Distance from point p to line (a, b).
     */
    double static distFromPointToLine(
        cv::Point3d p,
        cv::Point3d a,
        cv::Point3d b);

    double static length(Point3d p);

    Point3d static normalize(Point3d p);

    cv::Point2d static scaleToCameraUnits(Point2d p, Mat image);
};

#endif

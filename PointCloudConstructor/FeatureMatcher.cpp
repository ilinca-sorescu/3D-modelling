#include "FeatureMatcher.h"
#include "Image.h"
#include <cv.h>
#include <highgui.h>
#include <fstream>

using namespace std;
using namespace cv;

FeatureMatcher::FeatureMatcher(vector<shared_ptr<Image>> images) {
  this->images = images;
}

double FeatureMatcher::length(Point3d p) {
  return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

Point3d FeatureMatcher::normalize(Point3d p) {
  double len = length(p);
  return Point3d(p.x/len, p.y/len, p.z/len);
}

/*
 * Compute the distance from p to (a, b).
 */
double FeatureMatcher::distFromPointToLine(
    Point3d p,
    Point3d a,
    Point3d b) {

  //u is a unit vector in the direction of the line (a, b).
  Point3d u = normalize(b-a);
  return length(Point3d(p-a).cross(u));
}

//assumptions - the default height in povray is 1 (from -0.5 to 0.5).
Point2d FeatureMatcher::scaleToCameraUnits(
    Point2d p, Mat m) {
  double ratio = m.size().width/m.size().height;
  return Point2d(p.x/m.size().width*ratio,
                 p.y/m.size().height);
}

//p1 - the 2D point (in camera coordinates) corresponding to the location of
//     the feature with id m.queryIdx in img1.
//p2 - the 2D point (in camera coordinates) corresponding to the location of
//     the feature with id m.trainIdx in img2.
void FeatureMatcher::cameraCoordinatesOfDMatch(
    DMatch m,
    shared_ptr<Image> img1,
    shared_ptr<Image> img2,
    Point2d& p1,
    Point2d& p2) {

  KeyPoint feature1 = (img1->getFeatures())[m.queryIdx];
  KeyPoint feature2 = (img2->getFeatures())[m.trainIdx];

  Mat m1 = img1->getMat();
  Mat m2 = img2->getMat();
  //translate coordinate system to camera plane
  p1 = Point2d(feature1.pt.x - m1.size().width/2,
               feature1.pt.y - m1.size().height/2);
  p2 = Point2d(feature2.pt.x - m2.size().width/2,
               feature2.pt.y - m2.size().height/2);

  //scale to camera coordinates
  p1 = scaleToCameraUnits(p1, m1);
  p2 = scaleToCameraUnits(p2, m2);
}

/*
 * Returns only the matches which make epipolar sense.
 * See H&Z/244 (Chapter 9) for details.
 * matches - all of the matches have the same queryIdx and
 *           the same trainIdx !!!! (PRECONDITION).
 * img1 - image containing the query keypoint.
 * img2 - image containing the train keypoint.
 */
vector<DMatch> FeatureMatcher::filterMatches(
    vector<DMatch> matches,
    shared_ptr<Image> img1,
    shared_ptr<Image> img2) {

  float min_dist = matches[0].distance;
  for(unsigned int i = 1; i != matches.size(); i++ ) {
    if(matches[i].distance < min_dist)
      min_dist = matches[i].distance;
  }

  vector<DMatch> good_matches;

  Matx34d P1 = img1->getCameraMatrix();
  Matx34d P2 = img2->getCameraMatrix();
  Matx43d P1Plus; //pseudo inverse of P1
  invert(P1, P1Plus, DECOMP_SVD);
  Point3d cameraLoc1 = img1->getCameraPose();
  Matx41d C1(cameraLoc1.x, cameraLoc1.y, cameraLoc1.z, 1.0);
  Matx31d P2C1 = P2*C1;
  Point3d epipole2(P2C1(0), P2C1(1), P2C1(2));

  for(auto m:matches) {
    //find the locations of the features in img1 and img2
    Point2d p1, p2;
    cameraCoordinatesOfDMatch(m, img1, img2, p1, p2);

    //in homogeneous coordinates
    Matx31d x1(p1.x, p1.y, 1.0);
    Point3d x2(p2.x, p2.y, 1.0);

    //compute epipolar line in img2 corresponding to point x1 in img1.
    Matx31d aux = (P2*P1Plus)*x1;
    Point3d pointOnEpi(aux(0), aux(1), aux(2));

    /*
     * Check if x2 is on the epipolar line!
     * The epipolar line passes through the epipole and (P2)(P1+),
     * where P1+ is the pseudo-inverse of P1 (the camera matrix for img1).
     * See H&Z p. 244.
     * Note: the tolerance is needed because the pseudo-inverse
     * is an approximation.
     */

    //if makes epipolar sense
    if(distFromPointToLine(x2, epipole2, pointOnEpi) < tolerance &&
        //if the distance between the train and query features is acceptable
        m.distance <= 2*min_dist) //!!!maybe comment this
      good_matches.push_back(m);
  }
/*  float min_dist = matches[0].distance;
  for(unsigned int i = 1; i != matches.size(); i++ ) {
    if(matches[i].distance < min_dist)
      min_dist = matches[i].distance;
  }

  std::vector<DMatch> good_matches;
  for(unsigned int i = 0; i!= matches.size(); i++) {
    if(matches[i].distance <= 2*min_dist)
      good_matches.push_back(matches[i]);
  }
*/
//  cout<<"!! "<<matches.size()<<" "<<good_matches.size()<<endl;

  return good_matches;
}

vector<DMatch> FeatureMatcher::match(unsigned int index1, unsigned int index2, bool draw) {
  try {
    if(index1 >= images.size())
      throw new Exception();
    if(index2 >= images.size())
      throw new Exception();

    shared_ptr<Image> im1 = images[index1];
    shared_ptr<Image> im2 = images[index2];

    if(im1->getFeatures().empty())
      throw new Exception();
    if(im2->getFeatures().empty())
      throw new Exception();

    Mat descriptors1 = im1->getDescriptors();
    Mat descriptors2 = im2->getDescriptors();

    if(descriptors1.empty())
      throw new Exception();
    if(descriptors2.empty())
      throw new Exception();

    vector<DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);

    if(matches.empty())
      throw new Exception();

   vector<DMatch> good_matches = filterMatches(
        matches,
        im1,
        im2);

    if(draw) {
      Mat img_matches;
      drawMatches(
          im1->getMat(), im1->getFeatures(),
          im2->getMat(), im2->getFeatures(),
          good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
          vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
      ostringstream title;
      title << "Matches: "<<index1<<", "<<index2;
      imshow(title.str(), img_matches);
      waitKey(0);
      destroyWindow(title.str());
    }
    return good_matches;
  } catch(...) {
    vector<DMatch> empty;
    return empty;
  }
}


/*
 * Image.cpp
 *
 *  Created on: Dec 9, 2014
 *      Author: ailinca
 */
#include <fstream>
#include <cv.h>
#include "Image.h"
#include "opencv2/nonfree/features2d.hpp"
#include <cmath>

using namespace cv;
using namespace std;

shared_ptr<FeatureDetector> Image::fdetector;
shared_ptr<DescriptorExtractor> Image::dextractor;

Image::Image(string folder, int imgID) {
  ostringstream imageFileName;
  imageFileName<<folder<<"/"<<imgID<<".png";

  ostringstream textFileName;
  textFileName<<folder<<"/"<<imgID;

  try {
    cout<<imageFileName.str()<<"   ";
    imgMat = imread(imageFileName.str(), 0); //0 for greyscale
    cout<<imgMat.size()<<"   ";

		ifstream in(textFileName.str().c_str());
		double x, y, z;
	  in>>x>>y>>z;
    cerr<<x<<" "<<y<<" "<<z<<endl;
    cameraPose = Point3f(x, y, z);
    cameraMatrix = computeCameraMatrix(cameraPose);
	  in.close();
  } catch (Exception e) {
	  cout<<"Unable to open image file "<<imageFileName.str()<<" or text file "<<textFileName.str()<<endl;
    exit(0);
	}

  if(!fdetector)
    fdetector = make_shared<SiftFeatureDetector>();
  fdetector->detect(this->imgMat, this->features);
  cout<<this->features.size()<<endl;

  if(!dextractor)
    dextractor = make_shared<SiftDescriptorExtractor>();
  dextractor->compute(this->imgMat, this->features, descriptors);
}

Point3d Image::Normalize(Point3d v) {
  double len = sqrt(v.x*v.x +
      v.y*v.y +
      v.z*v.z);
  return Point3d(v.x/len, v.y/len, v.z/len);
}

Matx44d Image::computeCameraMatrix(Point3d cameraPose) {
  /*
   * The camera matrix for a camera placed in (0,0,0) looking along the
   * positive z-axis is:
   *    1 0 0 0
   *    0 1 0 0
   *    0 0 1 0
   * (see the camera pinhole model) [1].
   * The camera matrix of a camera placed in
   * (cameraPose.x, cameraPose.y, cameraPose.z) looking along the origin is:
   * [R|T] where T is the translation vector and R the rotation matrix describing
   * the camera's movement from [1].
   */

  Point3d L = Normalize(Point3d(-cameraPose.x, -cameraPose.y, -cameraPose.z));
  Point3d u(0.0, 1.0, 0.0); //the up direction
  Point3d s = Normalize(L.cross(u));
  Point3d uprime = s.cross(L);

  Matx33d R(s.x,      s.y,      s.z,
            uprime.x, uprime.y, uprime.z,
            -L.x,     -L.y,     -L.z);
  Matx31d C(-cameraPose.x, -cameraPose.y, -cameraPose.z);
  Matx31d t = R*C;
  return Matx44d(R.val[0], R.val[1], R.val[2], t.val[0],
                 R.val[3], R.val[4], R.val[5], t.val[1],
                 R.val[6], R.val[7], R.val[8], t.val[2],
                 0.0,      0.0,      0.0,      1.0);
  /*
  Matx13d T(-cameraPose.x, -cameraPose.y, -cameraPose.z);
  Matx33d vx(0,     0,     cosx,
             0,     0,    -cosy,
             -cosx, cosy,  0);
  Matx33d unit(1, 0, 0,
               0, 1, 0,
               0, 0, 1);
  Matx33d R = unit + vx + vx*vx*((1+cosz)/(1-cosz*cosz));
  return Matx34d(R.val[0], R.val[1], R.val[2], T.val[0],
                 R.val[3], R.val[4], R.val[5], T.val[1],
                 R.val[6], R.val[7], R.val[8], T.val[2]);*/
}

vector<KeyPoint> Image::getFeatures() const {
  return this->features;
}

Mat Image::getMat() const {
  return imgMat;
}

Mat Image::getDescriptors() const {
  return this->descriptors;
}

Point3d Image::getCameraPose() const {
  return cameraPose;
}

Matx44d Image::getCameraMatrix() const {
  return cameraMatrix;
}


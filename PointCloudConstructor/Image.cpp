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

Matx34d Image::computeCameraMatrix(Point3f cameraPose) {
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
  
  Matx13d T(-cameraPose.x, -cameraPose.y, -cameraPose.z);
  Matx33d vx(0,             0,            -cameraPose.x,
             0,             0,             cameraPose.y,
             cameraPose.x, -cameraPose.y,  0);
  Matx33d unit(1, 0, 0,
               0, 1, 0,
               0, 0, 1);
  Matx33d R = unit + vx + vx*vx*((1+cameraPose.z)/(1-cameraPose.z*cameraPose.z));
  return Matx34d(R.val[0], R.val[1], R.val[2], T.val[0],
                 R.val[3], R.val[4], R.val[5], T.val[1],
                 R.val[6], R.val[7], R.val[8], T.val[2]);
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

Point3f Image::getCameraPose() const {
  return cameraPose;
}

Matx34d Image::getCameraMatrix() const {
  return cameraMatrix;
}


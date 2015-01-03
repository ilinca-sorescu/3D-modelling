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


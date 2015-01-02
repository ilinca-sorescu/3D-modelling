/*
 * Image.cpp
 *
 *  Created on: Dec 9, 2014
 *      Author: ailinca
 */
#include <fstream>
#include <cv.h>
#include <highgui.h>
#include "Image.h"

using namespace cv;
using namespace std;

Image::Image(string folder,
    int imgID, 
    shared_ptr<FeatureDetector> fdetector) {
  ostringstream imageFileName;
  imageFileName<<folder<<imgID<<".png";

  ostringstream textFileName;
  textFileName<<folder<<imgID;

  try {
    imgMat = imread(imageFileName.str(), 0); //0 for greyscale

		ifstream in(textFileName.str().c_str());
		double x, y, z;
	  in>>x>>y>>z;
		cameraPose = Point3f(x, y, z);
	  in.close();
  } catch (Exception e) {
	  cout<<"Unable to open image file "<<imageFileName.str()<<" or text file "<<textFileName.str()<<endl;
    exit(0);
	}

  fdetector->detect(this->imgMat, this->features);
}

vector<KeyPoint> Image::getFeatures() {
  return this->features;
}

/*
 * Image.h
 *
 *  Created on: Dec 9, 2014
 *      Author: ailinca
 */

#include <cv.h>
#include <highgui.h>
#include <memory>

#ifndef IMAGE_H_
#define IMAGE_H_

using namespace std;
using namespace cv;

class Image {
	public:
    Image(string folder, int imgID);
    vector<KeyPoint> getFeatures();
    Point3f getCameraPose();
    static void setFeatureDetector(shared_ptr<FeatureDetector> fdetector);

  private:
		Mat imgMat;
    Point3f cameraPose;
    vector<KeyPoint> features;
    static shared_ptr<FeatureDetector> fdetector;
};


#endif /* IMAGE_H_ */

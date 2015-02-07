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
    vector<KeyPoint> getFeatures() const;
    Mat getDescriptors() const;
    Point3d getCameraPose() const;
    Mat getMat() const;
    Matx34d getCameraMatrix() const;
    static Matx34d computeCameraMatrix(Point3d);
  
  private:
		Mat imgMat;
    Point3d cameraPose;
    Matx34d cameraMatrix;
    vector<KeyPoint> features;
    Mat descriptors;
    static Point3d Normalize(Point3d);
    static shared_ptr<FeatureDetector> fdetector;
    static shared_ptr<DescriptorExtractor> dextractor;
};


#endif /* IMAGE_H_ */

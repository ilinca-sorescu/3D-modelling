/*
 * Image.h
 *
 *  Created on: Dec 9, 2014
 *      Author: ailinca
 */

#ifndef IMAGE_H_
#define IMAGE_H_

#include <fstream>
#include <cv.h>
#include <highgui.h>
//#include "Point3D.h"

using namespace cv;
using namespace std;

class Image {
	public:
		Image(string folder, int imgID) {
			ostringstream imageFileName;
			imageFileName<<folder<<imgID<<".png";

			ostringstream textFileName;
			textFileName<<folder<<imgID;

			try {
				imgMat = imread(imageFileName.str());

				ifstream in(textFileName.str().c_str());
				double x, y, z;
				in>>x>>y>>z;
				cameraPose = Point3d(x, y, z);
				in.close();
			} catch (Exception e) {
				cout<<"Unable to open image file "<<imageFileName.str()<<" or text file "<<textFileName.str()<<endl;
				exit(0);
			}
		}

		cv::Mat imgMat;
		Point3d cameraPose;
};


#endif /* IMAGE_H_ */

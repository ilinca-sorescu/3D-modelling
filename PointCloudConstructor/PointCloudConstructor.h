/*
 * PointCloudConstructor.h
 *
 *  Created on: Dec 8, 2014
 *      Author: ailinca
 */

#ifndef POINTCLOUDCONSTRUCTOR_H_
#define POINTCLOUDCONSTRUCTOR_H_

#include "Point3D.h"
#include "Image.h"
#include <memory>

class PointCloudConstructor {
	public:
		PointCloudConstructor(std::string folder);
    std::vector<Point3D> getPoints();

	private:
		std::vector<std::shared_ptr<Image>> images;
};


#endif /* POINTCLOUDCONSTRUCTOR_H_ */

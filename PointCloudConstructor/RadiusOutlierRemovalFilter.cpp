#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include "RadiusOutlierRemovalFilter.h"

using namespace pcl;

RadiusOutlierRemovalFilter::RadiusOutlierRemovalFilter(
    PointCloud<PointXYZRGB>::Ptr cloud) {
  filteredCloud = PointCloud<PointXYZRGB>::Ptr(
      new PointCloud<PointXYZRGB>);
  RadiusOutlierRemoval<PointXYZRGB> f;
  f.setInputCloud(cloud);
  f.setRadiusSearch(0.1);
  f.setMinNeighborsInRadius (4);
  f.filter (*filteredCloud);
  filteredCloud->height = 1;
  filteredCloud->width = filteredCloud->points.size();
}

PointCloud<PointXYZRGB>::Ptr RadiusOutlierRemovalFilter::getFilteredCloud() {
  return filteredCloud;
}


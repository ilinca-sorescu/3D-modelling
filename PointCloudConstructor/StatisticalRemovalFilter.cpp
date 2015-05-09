//based on http://pointclouds.org/documentation/tutorials/statistical_outlier.php
#include <pcl/filters/statistical_outlier_removal.h>
#include "StatisticalRemovalFilter.h"

using namespace pcl;

StatisticalRemovalFilter::StatisticalRemovalFilter(
    PointCloud<PointXYZRGB>::Ptr cloud) {
  filteredCloud = PointCloud<PointXYZRGB>::Ptr(
      new PointCloud<PointXYZRGB>);
  StatisticalOutlierRemoval<PointXYZRGB> f;
  f.setInputCloud(cloud);
  f.setMeanK(50);
  f.setStddevMulThresh(1.0);
  f.filter(*filteredCloud);
  filteredCloud->height = 1;
  filteredCloud->width = filteredCloud->points.size();
}

PointCloud<PointXYZRGB>::Ptr StatisticalRemovalFilter::getFilteredCloud() {
  return filteredCloud;
}

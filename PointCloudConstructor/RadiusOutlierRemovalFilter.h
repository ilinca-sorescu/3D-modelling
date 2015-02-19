#include<pcl/pcl_base.h>

class RadiusOutlierRemovalFilter {
  
  public:
    RadiusOutlierRemovalFilter(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getFilteredCloud();
  
  private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud;
};

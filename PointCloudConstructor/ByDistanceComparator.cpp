#include <memory>
#include <cv.h>
#include "Image.h"
#include "ByDistanceComparator.h"

float ByDistanceComparator::dist(Point3f a, Point3f b) {
  return (a.x-b.x)*(a.x-b.x) +
    (a.y-b.y)*(a.y-b.y) +
    (a.z-b.z)*(a.z-b.z);
}

ByDistanceComparator::ByDistanceComparator(
    int _refImg,
    const std::vector<std::shared_ptr<Image>> &imgs):
      refImg(_refImg), images(imgs) {
}

bool ByDistanceComparator::operator() (int i, int j) {
  Point3f p0 = images[refImg]->getCameraPose();
  Point3f p1 = images[i]->getCameraPose();
  Point3f p2 = images[j]->getCameraPose();
  return dist(p0, p1) < dist(p0, p2);
}



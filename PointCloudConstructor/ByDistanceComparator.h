#include <memory>
#include <cv.h>
#include "Image.h"

class ByDistanceComparator {

  private:
    int refImg;
    const std::vector<std::shared_ptr<Image>> &images;
    float dist(Point3f a, Point3f b);

  public:
    ByDistanceComparator(
        int _refImg,
        const std::vector<std::shared_ptr<Image>> &imgs);
    bool operator() (int i, int j);
};



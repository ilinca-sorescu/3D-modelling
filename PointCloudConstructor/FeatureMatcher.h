#ifndef FEATUREMATCHER_H_
#define FEATUREMATCHER_H_

#include "Image.h"
#include <memory>

class FeatureMatcher {
  public:
    FeatureMatcher(std::vector<std::shared_ptr<Image>>);
    std::vector<cv::DMatch> match(unsigned int, unsigned int, bool draw=false);

  private:
    std::vector<std::shared_ptr<Image>> images;
    FlannBasedMatcher matcher;
};

#endif

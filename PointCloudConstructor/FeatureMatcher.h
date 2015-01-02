#ifndef FEATUREMATCHEiR_H_
#define FEATUREMATCHER_H_

#include "Image.h"
#include <memory>

class FeatureMatcher {
  public:
    FeatureMatcher(std::vector<std::shared_ptr<Image>>);
};

#endif

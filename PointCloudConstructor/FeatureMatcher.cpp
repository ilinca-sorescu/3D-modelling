#include "FeatureMatcher.h"
#include "Image.h"
#include <cv.h>
#include <highgui.h>
#include <fstream>

using namespace std;
using namespace cv;

FeatureMatcher::FeatureMatcher(vector<shared_ptr<Image>> images) {
  this->images = images;
}

vector<DMatch> filteredMatches(vector<DMatch> matches) {
  return matches;
  float min_dist = matches[0].distance;
  for(unsigned int i = 1; i != matches.size(); i++ ) {
    if(matches[i].distance < min_dist)
      min_dist = matches[i].distance;
  }

  std::vector<DMatch> good_matches;
  for(unsigned int i = 0; i!= matches.size(); i++) {
    if(matches[i].distance <= 2*min_dist)
      good_matches.push_back(matches[i]);
  }

  return good_matches;
}

vector<DMatch> FeatureMatcher::match(unsigned int index1, unsigned int index2, bool draw) {
  assert(index1 < images.size() && index1 >= 0);
  assert(index2 < images.size() && index2 >= 0);

  shared_ptr<Image> im1 = images[index1];
  shared_ptr<Image> im2 = images[index2];

  assert(! im1->getFeatures().empty());
  assert(! im2->getFeatures().empty());

  Mat descriptors1 = im1->getDescriptors();
  Mat descriptors2 = im2->getDescriptors();

  assert(! descriptors1.empty());
  assert(! descriptors2.empty());

  vector<DMatch> matches;
  matcher.match(descriptors1, descriptors2, matches);

  assert(! matches.empty());

  vector<DMatch> good_matches = filteredMatches(matches);

  if(draw) {
    Mat img_matches;
    drawMatches(
        im1->getMat(), im1->getFeatures(),
        im2->getMat(), im2->getFeatures(),
        good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
        vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    ostringstream title;
    title << "Matches: "<<index1<<", "<<index2;
    imshow(title.str(), img_matches);
    waitKey(0);
    destroyWindow(title.str());
  }
  return good_matches;
}

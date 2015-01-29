#include <cv.h>
#include "PointCloudConstructor.h"
#include "Image.h"
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <cstdlib>
#include <algorithm>
#include "ByDistanceComparator.h"

using namespace std;
using namespace cv;
using namespace boost::filesystem;

PointCloudConstructor::PointCloudConstructor(string folder) {
  path p(folder);
	if(!is_directory(folder)) {
		cout<<"Error: unable to open input directory."<<endl;
		exit(1);
	}

  int numberOfPics = 0;
  for(const directory_entry& entry : directory_iterator(p))
    if(entry.path().extension() == ".png")
      ++numberOfPics;

  for(auto i = 0; i != numberOfPics; ++i)
    images.push_back(make_shared<Image>(folder, i));
  cout<<numberOfPics<<" images were successfully loaded!"<<endl;

  featureMatcher = unique_ptr<FeatureMatcher>(new FeatureMatcher(images));

  kclosest.resize(images.size());
  matches.resize(images.size());
  for(auto& m:matches)
    m.resize(K_);
}

void PointCloudConstructor::compute_kclosest() {
  //BRUTE FORCE -> replace later by KNN
  vector<int> indices;
  for(auto i=0u; i != images.size(); ++i)
    indices.push_back(i);
  for(auto i=0u; i != images.size(); ++i) {
    ByDistanceComparator cmp(i, images);
    sort(indices.begin(), indices.end(), cmp);
    cerr<<i<<" "; //DEBUG
    for(auto j=1; j <= K_; ++j) {
      kclosest[i].push_back(indices[j]);
      cerr<<indices[j]<<" "; //DEBUG
    }
    cerr<<endl; //DEBUG
  }
}

vector<Point3f> PointCloudConstructor::getPoints() {
  compute_kclosest();
  for(auto i=0u; i != images.size(); ++i) {

    //match features of i  against each of the closest K_ images
    for(auto j=0u; j != K_; ++j) {
      if(j == images.size())
        break;
      matches[i][j] = featureMatcher->match(i, kclosest[i][j]);
    }
  }

  //DEBUG
  featureMatcher->match(0, kclosest[0][0], true);

  return vector<Point3f>();
}

int main(int argc, char *argv[]) {
	string folder = argc>1? argv[1]:".";
  PointCloudConstructor *pcc = new PointCloudConstructor(folder);
  pcc->getPoints();
  delete pcc;
  return 0;
}



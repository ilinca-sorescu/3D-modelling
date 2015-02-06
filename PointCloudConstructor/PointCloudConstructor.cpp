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

vector<shared_ptr<Image>> PointCloudConstructor::getImages() {
  return images;
}

Point3d triangulate(DMatch match, Image img1, Image img2) {
  return ;
}

vector<Point3d> PointCloudConstructor::getPoints() {
  vector<Point3d> points3D;

  compute_kclosest();
  for(auto i=0u; i != images.size(); ++i) {

    //generatedPoints[x] = the 3D points resulted from triangulating
    //                     the xth feature of image i with the corresponding
    //                     feature in one of the kclosest images to i.
    vector<vector<Point3d>> generatedPoints;
    generatedPoints.resize(images[i]->getFeatures().size());

    //match features of i  against each of the closest K_ images
    for(auto j=0u; j != K_; ++j) {
      if(j == images.size())
        break;

      //index of jth closest image to i
      int jthImage = kclosest[i][j];
      matches[i][j] = featureMatcher->match(i, jthImage);
      vector<DMatch> &currentMatches = matches[i][j];

      for(const auto &m: currentMatches)
        generatedPoints[m.trainIdx].push_back(triangulate(m, images[i], images[jthImage]));

      //compute the average of the generatedPoints for each trainIdx
      for(auto gp:generatedPoints) {
        Point3d sum(0, 0, 0);
        for(auto p:gp)
          sum += p;
        points3D.push_back(Point3d(sum.x/gp.size(),
                                   sum.y/gp.size(),
                                   sum.z/gp.size()));
      }
    }
  }

  //DEBUG
  //featureMatcher->match(0, kclosest[0][0], true);

  return points3D;
}

int main(int argc, char *argv[]) {
	string folder = argc>1? argv[1]:".";

  /* camera matrix test -- should give (0,0,0)
  Matx44d p = Image::computeCameraMatrix(Point3d(1, 2, 3));
  cout<<p.val[0]<<" "<<p.val[1]<<" "<<p.val[2]<<" "<<p.val[4]<<endl;
  cout<<p.val[5]<<" "<<p.val[6]<<" "<<p.val[7]<<" "<<p.val[8]<<endl;
  cout<<p.val[9]<<" "<<p.val[10]<<" "<<p.val[11]<<" "<<p.val[12]<<endl;
  Matx41d r = p*Matx41d(1, 2, 3, 1);
  cout<<r.val[0]<<" "<<r.val[1]<<" "<<r.val[2]<<endl;*/

 PointCloudConstructor *pcc = new PointCloudConstructor(folder);
/*  vector<shared_ptr<Image>> imgs = pcc->getImages();
  float minx=15, miny=15, maxx=-15, maxy=-15;
  for(auto x:imgs) {
    for(auto cp:x->getFeatures()) {
      if(minx > cp.pt.x)
         minx = cp.pt.x;
      if(miny > cp.pt.y)
         miny = cp.pt.y;
       if(maxx < cp.pt.x)
         maxx = cp.pt.x;
      if(maxy < cp.pt.y)
         maxy = cp.pt.y;
    }
  }
  cout<<minx<<" "<<miny<<" "<<maxx<<" "<<maxy<<endl;
  */
  pcc->getPoints();
  delete pcc;
  return 0;
}



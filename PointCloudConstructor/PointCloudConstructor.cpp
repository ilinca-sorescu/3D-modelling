#include <cv.h>
#include "PointCloudConstructor.h"
#include "Image.h"
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <cstdlib>
#include <algorithm>
#include "ByDistanceComparator.h"
#include <fstream>

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
    //cerr<<i<<" "; //DEBUG
    for(auto j=1; j <= K_; ++j) {
      kclosest[i].push_back(indices[j]);
    //  cerr<<indices[j]<<" "; //DEBUG
    }
    //cerr<<endl; //DEBUG
  }
}

vector<shared_ptr<Image>> PointCloudConstructor::getImages() {
  return images;
}

Point3d PointCloudConstructor::triangulate(
    Matx34d C1, Matx34d C2, Point2d p1, Point2d p2) {
  Matx31d X;

  Matx43d A(p1.x*C1(2, 0)-C1(0, 0), p1.x*C1(2, 1)-C1(0, 1), p1.x*C1(2, 2)-C1(0, 2),
            p1.y*C1(2, 0)-C1(1, 0), p1.y*C1(2, 1)-C1(1, 1), p1.y*C1(2, 2)-C1(1, 2),
            p2.x*C2(2, 0)-C2(0, 0), p2.x*C2(2, 1)-C2(0, 1), p2.x*C2(2, 2)-C2(0, 2),
            p2.y*C2(2, 0)-C2(1, 0), p2.y*C2(2, 1)-C2(1, 1), p2.y*C2(2, 2)-C2(1, 2));

  Matx41d B(-p1.x*C1(2, 3)+C1(0, 3),
            -p1.y*C1(2, 3)+C1(1, 3),
            -p2.x*C2(2, 3)+C2(0, 3),
            -p2.y*C2(2, 3)+C2(1, 3));

  solve(A, B, X, DECOMP_SVD);

  return Point3d(X(0), X(1), X(2));
}

//assumptions - the default height in povray is 1 (from -0.5 to 0.5).
Point2d PointCloudConstructor::scaleToCameraUnits(
    Point2d p, Mat m) {
  double ratio = m.size().width/m.size().height;
  return Point2d(p.x/m.size().width*ratio,
                 p.y/m.size().height);
}

//p1 - the 2D point (in camera coordinates) corresponding to the location of
//     the feature with id m.queryIdx in img1.
//p2 - the 2D point (in camera coordinates) corresponding to the location of
//     the feature with id m.trainIdx in img2.
void PointCloudConstructor::imageCoordinatesOfDMatch(
    DMatch m,
    shared_ptr<Image> img1,
    shared_ptr<Image> img2,
    Point2d& p1,
    Point2d& p2) {

  KeyPoint feature1 = (img1->getFeatures())[m.queryIdx];
  KeyPoint feature2 = (img2->getFeatures())[m.trainIdx];

  Mat m1 = img1->getMat();
  Mat m2 = img2->getMat();
  //translate coordinate system to camera plane
  p1 = Point2d(feature1.pt.x - m1.size().width/2,
               feature1.pt.y - m1.size().height/2);
  p2 = Point2d(feature2.pt.x - m2.size().width/2,
               feature2.pt.y - m2.size().height/2);

  //scale to camera coordinates
  p1 = scaleToCameraUnits(p1, m1);
  p2 = scaleToCameraUnits(p2, m2);
}

vector<Point3d> PointCloudConstructor::getPoints() {
  cout<<"***Generating the point cloud***"<<endl;

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

      for(const auto &m: currentMatches) {
        Matx34d cameraM1 = images[i]->getCameraMatrix();
        Matx34d cameraM2 = images[jthImage]->getCameraMatrix();

        //Get the 2D locations(in image coordinates) corresponding to the current match
        Point2d p1, p2;
        imageCoordinatesOfDMatch(m, images[i], images[jthImage], p1, p2);

        generatedPoints[m.queryIdx].push_back(triangulate(cameraM1, cameraM2, p1, p2));
      }

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
  cout<<"The point cloud was successfully generated!"<<endl;
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
  vector<Point3d> points = pcc->getPoints();

  ofstream out;
  out.open("cloud.txt");
  for(auto p:points)
    out<<p.x<<" "<<p.y<<" "<<p.z<<endl;
  delete pcc;
/*  Matx43d A(2, 3, 4,
            5, 6, 7,
            8, 11, 12,
            12, 13, 14);
  Matx34d Ainv;
  invert(A, Ainv, DECOMP_SVD);*/
  return 0;
}



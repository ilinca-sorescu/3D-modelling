#include <cv.h>
#include "PointCloudConstructor.h"
#include "Image.h"
#include <boost/filesystem.hpp>
#include <boost/thread/executors/basic_thread_pool.hpp>
#include <boost/range/iterator_range.hpp>
#include <cstdlib>
#include <algorithm>
#include "ByDistanceComparator.h"
#include <fstream>
#include <pcl/pcl_base.h>
#include "StatisticalRemovalFilter.h"
#include "RadiusOutlierRemovalFilter.h"
#include <pcl/io/pcd_io.h>

#define white 16777215

using namespace std;
using namespace cv;
using namespace boost::filesystem;
using namespace pcl;

PointCloudConstructor::PointCloudConstructor(
    string folder,
    int wantedNumberOfPics) { //default: -1
  path p(folder);
	if(!is_directory(folder)) {
		cout<<"Error: unable to open input directory."<<endl;
		exit(1);
	}

  int numberOfPics = 0;
  for(const directory_entry& entry : directory_iterator(p))
    if(entry.path().extension() == ".png")
      ++numberOfPics;

  if(numberOfPics < wantedNumberOfPics) {
    cout<<"Sorry - There are not enough pictures in the "<<
      "specified folder."<<endl;
    exit(1);
  }

  if(wantedNumberOfPics != -1 &&
      numberOfPics > wantedNumberOfPics)
    numberOfPics = wantedNumberOfPics;

  images.resize(numberOfPics);
  boost::basic_thread_pool pool(8);
  for(auto i = 0; i != numberOfPics; ++i)
    pool.submit([this, i, folder]() {
      images[i] = make_shared<Image>(folder, i);});
  pool.close();
  pool.join();
  cout<<numberOfPics<<" images were successfully loaded!"<<endl;

  featureMatcher = unique_ptr<FeatureMatcher>(new FeatureMatcher(images));

  kclosest.resize(images.size());
  matches.resize(images.size());
  for(auto& m:matches)
    m.resize(K_);

//  normalizeCloudValues();

  populateCloud(this->getPoints());
  cout<<cloud->points.size()<<" points before radius outlier removal."<<endl;
  cloud = RadiusOutlierRemovalFilter(cloud).getFilteredCloud();
  cout<<cloud->points.size()<<" points after radius outlier removal."<<endl;

  cout<<cloud->points.size()<<" points before statistical removal."<<endl;
  cloud = StatisticalRemovalFilter(cloud).getFilteredCloud();
  cout<<cloud->points.size()<<" points after statistical removal."<<endl;

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

double inline abs(double x) {
  return x<0?-x:x;
}

double inline max(double x, double y) {
  return x<y?y:x;
}

void PointCloudConstructor::normalizeCloudValues() {
  double maxv = 0;
  for(auto p:cloud->points) {
      maxv = max(maxv,
        max(abs(p.x),
          max(abs(p.y),
            abs(p.z))));
  }
  cout<<maxv<<endl;
  for(auto p:cloud->points) {
      p.x/=maxv;
      p.y/=maxv;
      p.z/=maxv;
  }
}

vector<shared_ptr<Image>> PointCloudConstructor::getImages() {
  return images;
}

Point3d PointCloudConstructor::triangulate(
    Matx34d C1, Matx34d C2, Point2d p1, Point2d p2, int i, int j) {
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

  if(X(0) > 40 || X(0) < -40 ||
     X(1) > 40 || X(1) < -40 ||
     X(2) > 40 || X(2) < -40) {
    cout<<"point3d: "<<X(0)<<" "<<X(1)<<" "<<X(2)<<endl;
    cout<<"point2d: "<<p1.x<<" "<<p1.y<<" "<<i<<" "<<
      reprojectionError(C1, p1, Point3d(X(0), X(1), X(2)))<<endl;
    cout<<"point2d: "<<p2.x<<" "<<p2.y<<" "<<j<<" "<<
      reprojectionError(C2, p2, Point3d(X(0), X(1), X(2)))<<endl;
    Matx41d R = A*X;
    cout<<R(0)<<" "<<R(1)<<" "<<R(2)<<" "<<R(3)<<endl;
    cout<<B(0)<<" "<<B(1)<<" "<<B(2)<<" "<<B(3)<<endl<<endl;
  }

  return Point3d(X(0), X(1), X(2));
}

vector<Point3d> PointCloudConstructor::filterByReprojectionError(
    vector<pair<Point3d, double>> points) {

  //replace with statistical filtering if necessary
  vector<Point3d> good_points;
  for(auto i = 0u; i != points.size(); ++i)
    //if the reprojection error is small
    if(points[i].second <= 0.0001)
      good_points.push_back(points[i].first);

  cout<<"Before filtering by reprojection error: "<<points.size()<<endl;
  cout<<"After filtering by reprojection error: "<<good_points.size()<<endl;

  return good_points;
}

double distance(Point2d a, Point2d b) {
  Point2d p = a-b;
  return sqrt(p.x*p.x + p.y*p.y);
}

double PointCloudConstructor::reprojectionError(
    Matx34d CameraMat,
    Point2d x,
    Point3d X) {

  //compute error in equ. x = Camera * X (in homogeneous coords.)
  Matx41d XHomogeneous(X.x, X.y, X.z, 1);
  Matx31d result = CameraMat * XHomogeneous;
  Point2d xReprojected(result(0)/result(2),
                       result(1)/result(2));
  return distance(x, xReprojected);
}

vector<Point3d> PointCloudConstructor::getPoints() {
  cout<<"***Generating the point cloud***"<<endl;

  vector<pair<Point3d, double>> points3D;

  compute_kclosest();
  for(auto i=0u; i != images.size(); ++i) {

    //generatedPoints[x] = the 3D points resulted from triangulating
    //                     the xth feature of image i with the corresponding
    //                     feature in one of the kclosest images to i.
    vector<vector<pair<Point3d, double>>> generatedPoints;
    generatedPoints.resize(images[i]->getFeatures().size());

    //match features of i  against each of the closest K_ images
    for(auto j=0u; j != K_; ++j) {
      if(j == images.size())
        break;

      //index of jth closest image to i
      int jthImage = kclosest[i][j];
      matches[i][j] = featureMatcher->match(i, jthImage);//, true); //delete true!
      vector<DMatch> &currentMatches = matches[i][j];

      for(const auto &m: currentMatches) {
        Matx34d cameraM1 = images[i]->getCameraMatrix();
        Matx34d cameraM2 = images[jthImage]->getCameraMatrix();

        //Get the 2D locations(in image coordinates) corresponding to the current match
        Point2d p1, p2;
        FeatureMatcher::cameraCoordinatesOfDMatch(m, images[i], images[jthImage], p1, p2);

        Point3d X = triangulate(cameraM1, cameraM2, p1, p2, i, jthImage);
        generatedPoints[m.queryIdx].push_back(make_pair(
            X,
            max(reprojectionError(cameraM1, p1, X),
                reprojectionError(cameraM2, p2, X))));
      }

      //compute the average of the generatedPoints for each trainIdx
      for(auto gp:generatedPoints) {
        Point3d sum3D(0, 0, 0);
        double errorSum = 0;
        for(auto p:gp) {
          sum3D += p.first;
          errorSum += p.second;
        }
        if(gp.size() != 0) {
          Point3d X(sum3D.x/gp.size(),
                    sum3D.y/gp.size(),
                    sum3D.z/gp.size());
          double reprojectionError = errorSum/gp.size();
          points3D.push_back(make_pair(X, reprojectionError));
        }
      }
    }
  }

  //DEBUG
  featureMatcher->match(0, kclosest[0][0], true);
  cout<<"The point cloud was successfully generated!"<<endl;
  return filterByReprojectionError(points3D);
}

void PointCloudConstructor::populateCloud(
    vector<Point3d> points) {
  cloud.reset(new PointCloud<PointXYZRGB>);
  cerr<<"Number of points: "<<points.size()<<endl;
  for(auto i:points) {
    PointXYZRGB p;
    p.x = i.x;
    p.y = i.y;
    p.z = i.z;
    p.rgb = white;
    cloud->push_back(p);
  }
  cloud->height = 1;
  cloud->width = (uint32_t)cloud->points.size();
}

void PointCloudConstructor::cloudToPCD(string outFile) {
  pcl::io::savePCDFileASCII(outFile, *cloud);
}

void PointCloudConstructor::cloudToTxt(String outFile) {
  ofstream  out(outFile);
  for (auto i=0u; i < cloud->points.size(); i++) {
    PointXYZRGB p = cloud->points[i];
    out << p.x << " " << p.y << " " << p.z << endl;
  }
  out.close();
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

  int NumberOfPics = -1;
  try {
    if(argc > 2)
      NumberOfPics = stoi(argv[2]);
    if(NumberOfPics < 4)
      throw new Exception();
  } catch(...){
    cout<<"The second argument represents the maximum number "<<
      "of pictures taken as input. "<<
      "This number must be greater than 4."<<endl;
    return 0;
  }
  PointCloudConstructor *pcc;
  if(argc < 2)
    pcc = new PointCloudConstructor(folder);
  else
    pcc = new PointCloudConstructor(folder, NumberOfPics);
  pcc->cloudToTxt("cloud.txt");
  pcc->cloudToPCD("cloud.pcd");
  delete pcc;
//  PointCloudConstructor *pcc;
//  if(argc < 2)
//    pcc = new PointCloudConstructor(folder);
//    pcc = new PointCloudConstructor(folder, NumberOfPics);
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
  //vector<Point3d> points = pcc->getPoints();
//  exportPCLCloud(
//      statisticalRemovalFilter(
//        populatePCLCloud(pcc->getPoints())),
//      "cloud.PCD");
/*  ofstream out;
  out.open("cloud.txt");
  for(auto p:points)
    out<<p.x<<" "<<p.y<<" "<<p.z<<endl;
 */
//  delete pcc;
/*  Matx43d A(2, 3, 4,
            5, 6, 7,
            8, 11, 12,
            12, 13, 14);
  Matx34d Ainv;
  invert(A, Ainv, DECOMP_SVD);*/
  return 0;
}



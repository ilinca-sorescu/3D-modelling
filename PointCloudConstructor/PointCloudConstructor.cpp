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
#include <mutex>

#define white 16777215
#define epsilon 0.0001

using namespace std;
using namespace cv;
using namespace boost::filesystem;
using namespace pcl;

double PointCloudConstructor::MinRatio = 0.0524;
double PointCloudConstructor::MaxRatio = 0.2;
double PointCloudConstructor::ReprojectionError = 0.000008;
double PointCloudConstructor::Tolerance = 0.001;
string PointCloudConstructor::OutputFile = "cloud";

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

  populateCloud(sortPoints(this->computePoints()));
  cout<<cloud->points.size()<<" points before radius outlier removal."<<endl;
  cloud = RadiusOutlierRemovalFilter(cloud).getFilteredCloud();
  cout<<cloud->points.size()<<" points after radius outlier removal."<<endl;

  cout<<cloud->points.size()<<" points before statistical removal."<<endl;
  cloud = StatisticalRemovalFilter(cloud).getFilteredCloud();
  cout<<cloud->points.size()<<" points after statistical removal."<<endl;

  normalizeCloudValues();
}

bool comparePoints(Point3d p1, Point3d p2) {
  return p1.y < p2.y;
}

vector<Point3d> PointCloudConstructor::sortPoints(vector<Point3d> points) {
  sort(points.begin(), points.end(), comparePoints);
  return points;
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

double inline absv(double x) {
  return x<0?-x:x;
}

double inline max(double x, double y) {
  return x<y?y:x;
}

void PointCloudConstructor::normalizeCloudValues() {
  double maxv = 0;
  for(auto p:cloud->points) {
      maxv = max(maxv,
        max(absv(p.x),
          max(absv(p.y),
            absv(p.z))));
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

  //solve(A, B, X, DECOMP_SVD);

  X = (A.t()*A).inv()*A.t()*B;


 /* if(X(0) > 10 || X(0) < -10 ||
     X(1) > 10 || X(1) < -10 ||
     X(2) > 10 || X(2) < -10) {
    cout<<"point3d: "<<X(0)<<" "<<X(1)<<" "<<X(2)<<endl;
    cout<<"point2d: "<<p1.x<<" "<<p1.y<<" "<<i<<" "<<
      reprojectionError(C1, p1, Point3d(X(0), X(1), X(2)))<<endl;
    cout<<"point2d: "<<p2.x<<" "<<p2.y<<" "<<j<<" "<<
      reprojectionError(C2, p2, Point3d(X(0), X(1), X(2)))<<endl;
    Matx41d R = A*X;
    cout<<R(0)<<" "<<R(1)<<" "<<R(2)<<" "<<R(3)<<endl;
    cout<<B(0)<<" "<<B(1)<<" "<<B(2)<<" "<<B(3)<<endl<<endl;
  }
*/
  return Point3d(X(0), X(1), X(2));
}

double distance(Point3d a, Point3d b) {
  return sqrt((a.x-b.x)*(a.x-b.x) +
              (a.y-b.y)*(a.y-b.y) +
              (a.z-b.z)*(a.z-b.z));
}

Point3d PointCloudConstructor::ReadjustedTriangulation(
    Matx34d C1, Matx34d C2, Point2d p1, Point2d p2, int i, int j) {


  Point3d initialX = triangulate(C1, C2, p1, p2, i, j);
  Matx31d X(initialX.x, initialX.y, initialX.z);
  int noOfIterations = 0;
  double w1 = 1.0, w2 = 1.0;
  while(noOfIterations < 10) {
    noOfIterations++;

    double newW1 = C1(2, 0)*X(0) + C1(2, 1)*X(1) + C1(2, 2)*X(2) + C1(2, 3)*X(3);
    double newW2 = C2(2, 0)*X(0) + C2(2, 1)*X(1) + C2(2, 2)*X(2) + C2(2, 3)*X(3);

    if(absv(w1-newW1) < epsilon && absv(w2-newW2) < epsilon)
      break;

    w1 = newW1;
    w2 = newW2;

    //readjust the weights of the two equations
    Matx43d A((p1.x*C1(2, 0)-C1(0, 0))/w1, (p1.x*C1(2, 1)-C1(0, 1))/w1, (p1.x*C1(2, 2)-C1(0, 2))/w1,
              (p1.y*C1(2, 0)-C1(1, 0))/w1, (p1.y*C1(2, 1)-C1(1, 1))/w1, (p1.y*C1(2, 2)-C1(1, 2))/w1,
              (p2.x*C2(2, 0)-C2(0, 0))/w2, (p2.x*C2(2, 1)-C2(0, 1))/w2, (p2.x*C2(2, 2)-C2(0, 2))/w2,
              (p2.y*C2(2, 0)-C2(1, 0))/w2, (p2.y*C2(2, 1)-C2(1, 1))/w2, (p2.y*C2(2, 2)-C2(1, 2))/w2);

    Matx41d B((-p1.x*C1(2, 3)+C1(0, 3))/w1,
              (-p1.y*C1(2, 3)+C1(1, 3))/w1,
              (-p2.x*C2(2, 3)+C2(0, 3))/w2,
              (-p2.y*C2(2, 3)+C2(1, 3))/w2);

//    Matx31d aux;
//    solve(A, B, aux, DECOMP_SVD);
    X = (A.t()*A).inv()*A.t()*B;

//    double error = max(reprojectionError(C1, p1, Point3d(X(0), X(1), X(2))),
//                     reprojectionError(C2, p2, Point3d(X(0), X(1), X(2))));

 /*   if((initialX.x > 12 || initialX.x < -12 ||
        initialX.y > 12 || initialX.y < -12 ||
        initialX.z > 12 || initialX.z < -12)) {
      Matx41d R = A*X;
      cout<<"initialX: "<<initialX.x<<" "<<initialX.y<<" "<<initialX.z<<endl;
      cout<<"point3d: "<<X(0)<<" "<<X(1)<<" "<<X(2)<<endl;
      cout<<"point2d: "<<p1.x<<" "<<p1.y<<" "<<i<<" "<<
        reprojectionError(C1, p1, Point3d(X(0), X(1), X(2)))<<endl;
      cout<<"point2d: "<<p2.x<<" "<<p2.y<<" "<<j<<" "<<
        reprojectionError(C2, p2, Point3d(X(0), X(1), X(2)))<<endl;
      cout<<"distance: "<<distance(images[i]->getCameraPose(),
                                   images[j]->getCameraPose())<<endl;
      cout<<"AX: "<<R(0)<<" "<<R(1)<<" "<<R(2)<<" "<<R(3)<<endl;
      cout<<"B: "<<B(0)<<" "<<B(1)<<" "<<B(2)<<" "<<B(3)<<endl;
      cout<<"A: "<<endl;
      cout<<A(0, 0)<<" "<<A(0, 1)<<" "<<A(0, 2)<<endl;
      cout<<A(1, 0)<<" "<<A(1, 1)<<" "<<A(1, 2)<<endl;
      cout<<A(2, 0)<<" "<<A(2, 1)<<" "<<A(2, 2)<<endl;
      cout<<A(3, 0)<<" "<<A(3, 1)<<" "<<A(3, 2)<<endl<<endl;
    }*/
  //  cout<<"reweighted X: "<<X(0)<<" "<<X(1)<<" "<<X(2)<<" "<<w1<<" "<<w2<<endl;
  }

  /*if(noOfIterations > 1) {
    cout<<"hola: "<<noOfIterations<<endl;
    cout<<tX.x<<" "<<tX.y<<" "<<tX.z<<endl;
    cout<<X(0)<<" "<<X(1)<<" "<<X(2)<<endl<<endl;
  }
  */
  return Point3d(X(0), X(1), X(2));
}

vector<Point3d> PointCloudConstructor::filterByReprojectionError(
    vector<pair<Point3d, double>> points) {

  //replace with statistical filtering if necessary
  vector<Point3d> good_points;
  for(auto i = 0u; i != points.size(); ++i)
    //if the reprojection error is small
    if(points[i].second <= ReprojectionError) //changed this from 0.0003
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

//decides which strategy is used for computing points and filter
vector<Point3d> PointCloudConstructor::computePoints() {
  return filterByReprojectionError(computePoints_ratioInterval());//_kclosest());
}

vector<PointCloudConstructor::TriangulatedPoint> PointCloudConstructor::computePointsBetweenTwoImgs(int i, int j) {
      vector<DMatch> matches = featureMatcher->match(i, j);

      Matx34d cameraM1 = images[i]->getCameraMatrix();
      Matx34d cameraM2 = images[j]->getCameraMatrix();

      vector<TriangulatedPoint> tPoints;

      for(const auto &m: matches) {

        //Get the 2D locations(in image coordinates) corresponding to the current match
        Point2d p1, p2;
        FeatureMatcher::cameraCoordinatesOfDMatch(m, images[i], images[j], p1, p2);

        TriangulatedPoint tp;
        tp.point = ReadjustedTriangulation(cameraM1, cameraM2, p1, p2, i, j);
        tp.reprojectionError = max(reprojectionError(cameraM1, p1, tp.point),
                                   reprojectionError(cameraM2, p2, tp.point));
        tp.imgId1 = i;
        tp.imgId2 = j;
        tp.featureId1 = m.queryIdx;
        tp.featureId2 = m.trainIdx;
        tPoints.push_back(tp);
      }

      return tPoints;
}

vector<pair<Point3d, double>> PointCloudConstructor::computePoints_kclosest() {
  cout<<"***Generating the point cloud***"<<endl;

  vector<pair<Point3d, double>> points3D;

  compute_kclosest();

 /*double s=0, min=15, maxi=0, d;
  vector<pair<double, int>> distances;
  for(auto i=0u; i != images.size(); ++i) {
    d = distance(images[i]->getCameraPose(),
                 images[kclosest[i][0]]->getCameraPose());
    s += d;
    if(min > d)
      min = d;
    if(maxi < d)
      maxi = d;
    distances.push_back(make_pair(d, i));
  }
  sort(distances.begin(), distances.end());
  for(auto dist:distances) {
    cout<<dist.first<<" "<<dist.second<<endl;
  }
  cout<<s/images.size()<<" "<<maxi<<" "<<min<<" "<<
    distance(images[306]->getCameraPose(),
             images[984]->getCameraPose())<<endl;
*/

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
      vector<TriangulatedPoint> tPoints = computePointsBetweenTwoImgs(i, jthImage);
      for(auto tp:tPoints)
        generatedPoints[tp.featureId1].push_back(make_pair(
              tp.point,
              tp.reprojectionError));
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

  //DEBUG
  //featureMatcher->match(984, kclosest[984][0], true);
  cout<<"The point cloud was successfully generated (using Kclosest)!"<<endl;
  return points3D;//filterByReprojectionError(points3D);
}

vector<pair<Point3d, double>> PointCloudConstructor::computePoints_ratioInterval() {
  cout<<"***Generating the point cloud***"<<endl;

  vector<pair<Point3d, double>> points3D;
 // double sum = 0;
 // int num = 0;
  double radius = distance(images[1]->getCameraPose(), Point3d(0, 0, 0));

  mutex push_backMutex;

  boost::basic_thread_pool pool(8);
  for(auto p1 = 0u; p1 != images.size(); ++p1)
    pool.submit([this, p1, radius, &points3D, &push_backMutex]() {
      //p2 = index of the closest image to p1 with ratio greater than MinRatio
     int p2;

      //smallest distance whose ratio is greater than MinRatio
      // double minDist = numeric_limits<double>::max();

      for(auto i = 0u; i != images.size(); ++i) {
        if(i == p1)
          continue;
        double dist = distance(images[p1]->getCameraPose(),
                               images[i]->getCameraPose());
    /*  if(dist < minDist && dist/radius > MinRatio) {
          minDist = dist;
          p2 = i;
        }
      }
      if(minDist/radius > MaxRatio)
       continue;
      cout<<p1<<" "<<p2<<" "<<minDist<<endl;
      sum += minDist;
      ++num;*/
      if(!(dist/radius > MinRatio && dist/radius < MaxRatio)) continue;
      p2 = i;
      //cout<<p1<<" "<<p2<<" "<<endl;
      vector<TriangulatedPoint> triangulatedPoints = computePointsBetweenTwoImgs(p1, p2);
        push_backMutex.lock();
      for(auto tp:triangulatedPoints)
        points3D.push_back(make_pair(tp.point, tp.reprojectionError));
      push_backMutex.unlock();
    }
  });
  pool.close();
  pool.join();

  //  cout<<"mean dist: "<<sum/num<<endl;
  cout<<MinRatio*radius<<endl;
  cout<<"The point cloud was successfully generated (using ratio interval)!"<<endl;
  return points3D;
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

void getParameterConfiguration(string configFile) {
  ifstream in(configFile.c_str());
  double minRatio, maxRatio, reprojectionError, tolerance;
  string outfile;
  in>>minRatio
    >>maxRatio
    >>reprojectionError
    >>tolerance
    >>outfile;

  PointCloudConstructor::MinRatio = minRatio;
  PointCloudConstructor::MaxRatio = maxRatio;
  PointCloudConstructor::ReprojectionError = reprojectionError;
  PointCloudConstructor::Tolerance = tolerance;
  PointCloudConstructor::OutputFile = outfile;
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
  string configFile;
  try {
    NumberOfPics = stoi(argv[2]);
    if(NumberOfPics < 4)
      throw new Exception();
   // if(argc > 3)
      configFile=argv[3];
  } catch(...){
    cout<<"The second argument represents the maximum number "<<
      "of pictures taken as input. "<<
      "This number must be greater than 4."<<endl;
    return 0;
  }

  //if(argc > 3)
    getParameterConfiguration(configFile);

  PointCloudConstructor *pcc;
  pcc = new PointCloudConstructor(folder, NumberOfPics);
  pcc->cloudToTxt(PointCloudConstructor::OutputFile);
  pcc->cloudToPCD(PointCloudConstructor::OutputFile + ".pcd");



 /* Matx34d C1 = pcc->getImages()[306]->getCameraMatrix();
  Point3d p1 = pcc->getImages()[306]->getCameraPose();
  Matx31d R1 = C1*Matx41d(p1.x, p1.y, p1.z, 1);
  cout<<"p1: "<<p1.x<<" "<<p1.y<<" "<<p1.z<<endl;
  cout<<"C1: "<<C1(0, 0)<<" "<<C1(0, 1)<<" "<<C1(0, 2)<<" "<<C1(0, 3)<<endl
              <<C1(1, 0)<<" "<<C1(1, 1)<<" "<<C1(1, 2)<<" "<<C1(1, 3)<<endl
              <<C1(2, 0)<<" "<<C1(2, 1)<<" "<<C1(2, 2)<<" "<<C1(2, 3)<<endl;
  cout<<"R1: "<<R1(0, 0)<<" "<<R1(1, 0)<<" "<<R1(2, 0)<<endl<<endl;


  Matx34d C2 = pcc->getImages()[984]->getCameraMatrix();
  Point3d p2 = pcc->getImages()[984]->getCameraPose();
  Matx31d R2 = C2*Matx41d(p2.x, p2.y, p2.z, 1);
  cout<<"p2: "<<p2.x<<" "<<p2.y<<" "<<p2.z<<endl;
  cout<<"C2: "<<C2(0, 0)<<" "<<C2(0, 1)<<" "<<C2(0, 2)<<" "<<C2(0, 3)<<endl
              <<C2(1, 0)<<" "<<C2(1, 1)<<" "<<C2(1, 2)<<" "<<C2(1, 3)<<endl
              <<C2(2, 0)<<" "<<C2(2, 1)<<" "<<C2(2, 2)<<" "<<C2(2, 3)<<endl;
  cout<<"R2: "<<R2(0, 0)<<" "<<R2(1, 0)<<" "<<R2(2, 0)<<endl<<endl;
*/
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
  //vector<Point3d> points = pcc->computePoints();
//  exportPCLCloud(
//      statisticalRemovalFilter(
//        populatePCLCloud(pcc->computePoints())),
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



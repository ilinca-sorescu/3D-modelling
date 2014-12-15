#include <cv.h>
#include "PointCloudConstructor.h"
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <cstdlib>

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
  for(int i = 0; i != numberOfPics; ++i)
    images.push_back(make_shared<Image>(folder, i));
  cout<<numberOfPics<<" images were successfully loaded!"<<endl;
}

vector<Point3D> PointCloudConstructor::getPoints() {
	vector<Point3D> a;
	return a;
}

int main(int argc, char *argv[]) {
	string folder = argc>1? argv[1]:".";
	PointCloudConstructor *pcc = new PointCloudConstructor(folder);
	delete pcc;
  return 0;
}



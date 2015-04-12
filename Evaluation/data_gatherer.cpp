#include<iostream>
#include<fstream>

using namespace std;

const double stepMinRatio = 0.01;
const double stepMaxRatio = 0.05;
const double stepReprojError = 0.000001;
const double stepTolerance = 0.0001;
const int smallStepNumPics = 100;
const int bigStepNumPics = 500;

const string pathToPCC = "~/Workspace/Project/3D-modelling/PointCloudConstructor/";
const string pathToResults =  "~/Workspace/Project/3D-modelling/Evaluation/Data/";

string createConfigFile(
    double minRatio,
    double maxRatio,
    double reprojectionError,
    double tolerance,
    string inputFolder,
    int numPics) {
  int randomId = rand();
  string configFileName = pathToResults + std::to_string(randomId);
  string outputFileName = configFileName + "_cloud";
  ofstream ofs(configFileName);
  ofs<<minRatio<<endl
     <<maxRatio<<endl
     <<reprojectionError<<endl
     <<tolerance<<endl
     <<outputFileName<<endl
     <<inputFolder<<endl
     <<numPics;
  ofs.close();
  return configFileName;
}

void gather(string inputFolder) {
  int currentStep = smallStepNumPics;

  for(int numPics = 100; numPics <= 3000; numPics += currentStep) {

    double reprojectionError = 0.000008;
    double tolerance = 0.001;
    for(double minRatio = 0; minRatio != 0.1; minRatio += stepMinRatio)
      for(double maxRatio = 0.05; maxRatio != 0.5; maxRatio += stepMaxRatio) {
        string configFileName = createConfigFile(
            minRatio,
            maxRatio,
            reprojectionError,
            tolerance,
            inputFolder,
            numPics);
        runPCC(inputFolder, numPics, configFileName);
      }

    if(numPics > 500)
      currentStep = bigStepNumPics;
  }
}

int main(int argc, char* argv[]) {
  string inputFolder = argv[1]; //absolute path! - always

  gather(inputFolder);

  return 0;
}

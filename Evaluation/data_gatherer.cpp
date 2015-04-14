#include<iostream>
#include<fstream>
#include<cstdio>

using namespace std;

const double stepMinRatio = 0.01;
const double stepMaxRatio = 0.05;
const int smallStepNumPics = 50;
const int bigStepNumPics = 500;

const double eps = 0.0000001;

const string pathToPCC = "~/Workspace/Project/3D-modelling/PointCloudConstructor/";
const string pathToResults =  "~/Workspace/Project/3D-modelling/Evaluation/Data/";
const string logFile = "Log";

string createConfigFile(
    double minRatio,
    double maxRatio,
    double reprojectionError,
    double tolerance,
    string inputFolder,
    int numPics) {
  int randomId = rand();
  string configFileName = "Data/" + std::to_string(randomId);
  string outputFileName = configFileName + "_cloud";
  cout<<configFileName<<": "<<configFileName<<endl;
  ofstream ofs;
  ofs.open(configFileName, ios::out);
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

string execute(char* cmd) {
  FILE* pipe = popen(cmd, "r");
  if(!pipe)
    return "ERROR";
  char buffer[128];
  string result = "\n";
  while(!feof(pipe)) {
    if(fgets(buffer, 128, pipe) != NULL)
      result += buffer;
  }
  pclose(pipe);
  return result;
}

void runPCC(
    string inputFolder,
    int numPics,
    string configFileName) {
  cout<<configFileName<<": ";
  string command = "time -o " + pathToResults + "time" + pathToPCC + "PointCloudConstructor "
    + inputFolder + " "
    + to_string(numPics) + " "
    + configFileName;
  cout<<command<<endl;
  string msg = execute((char*)command.c_str());

  ofstream logging;
  logging.open(logFile, std::ios_base::app);
  logging<<configFileName<<" "<<msg<<endl;
  logging.close();
}

void gather(string inputFolder) {
  int currentStep = smallStepNumPics;

  for(int numPics = 50; numPics <= 3000; numPics += currentStep) {
    for(double reprojectionError = 0.1; reprojectionError >= eps; reprojectionError /= 10)
      for(double tolerance = 0.1; tolerance >= eps; tolerance /= 10)
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
  if(argc < 1) {
    cout<<"introduce abs path to input folder!"<<endl;
    return 1;
  }
  string inputFolder = argv[1]; //absolute path! - always

  gather(inputFolder);

  return 0;
}

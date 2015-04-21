#include<iostream>
#include<fstream>
#include<cstdio>
#include<fstream>

using namespace std;

const double stepMinRatio = 0.02;
const double stepMaxRatio = 0.1;
const int smallStepNumPics = 50;
const int bigStepNumPics = 500;

const double eps = 0.00000001;

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
  string command = "time ("
    + pathToPCC
    + "PointCloudConstructor "
    + inputFolder + " "
    + to_string(numPics) + " "
    + configFileName + ") " +
    "2> " + configFileName + "_time";
  cout<<command<<endl;
  string msg = execute((char*)command.c_str());

  ofstream logging;
  logging.open(logFile, std::ios_base::app);
  logging<<configFileName<<" "<<msg<<endl;
  logging.close();
}

void gather(string inputFolder) {
  int currentStep = bigStepNumPics;

  for(int numPics = 2200; numPics >= 100; numPics -= 100) {
   // for(double minRatio = 0; minRatio <= 0.1; minRatio += stepMinRatio)
    //  for(double maxRatio = 0.5; maxRatio >= 0.05; maxRatio -= stepMaxRatio)
     //   for(double reprojectionError = 0.01; reprojectionError >= eps; reprojectionError /= 10) {
     //     int nonempty = false;
      //    for(double tolerance = 0.01; tolerance >= eps; tolerance /= 10) {
	double minRatio = 0.04;
	double maxRatio = 0.5;
	double reprojectionError = 0.0001;
	double tolerance = 0.001;

            string configFileName = createConfigFile(
                minRatio,
                maxRatio,
                reprojectionError,
                tolerance,
                inputFolder,
                numPics);
            runPCC(inputFolder, numPics, configFileName);
            ifstream cloudFile((configFileName+"_cloud").c_str());
            /*if(cloudFile.peek() == std::ifstream::traits_type::eof())
              break; //cloud file is empty
            nonempty = true;

          }
          if(nonempty == false)
            break;
    }
    if(numPics <= 500)
      currentStep = smallStepNumPics;*/
  }
}

int main(int argc, char* argv[]) {
  srand(time(NULL));
  if(argc < 2) {
    cout<<"introduce abs path to input folder!"<<endl;
    return 1;
  }
  string inputFolder = argv[1]; //absolute path! - always

  gather(inputFolder);

  return 0;
}

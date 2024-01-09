#include <iostream>
#include <yaml-cpp/yaml.h>

class BuildFromYaml {

  // read in file
  //parsing
  // create classes
  //assigning those classes into a robot -> return a robot

 private:
  void ReadFile();

 public:
  BuildFromYaml();
  void BuildRobot(std::string path);

};


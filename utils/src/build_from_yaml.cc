#include "huron/utils/build_from_yaml.h"

BuildFromYaml::BuildFromYaml() {
}

void BuildFromYaml::BuildRobot(std::string path) {
//  YAML::Emitter out;
//  out << "Hello, World!";
//  std::cout << "Here's the output YAML:\n" << out.c_str(); // prints "Hello, World!"

YAML::Node config = YAML::LoadFile(path);
// this needs the conversion from Node to double
//double kGearRatio1 = config["kGearRatio1"].as<double>();
//std::cout << kGearRatio1;

}


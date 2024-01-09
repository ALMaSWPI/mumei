#include "huron/utils/build_from_yaml.h"
#include <iostream>
int main() {
  BuildFromYaml builder;
  builder.BuildRobot("test_configuration_build.yaml");
  return 0;
}
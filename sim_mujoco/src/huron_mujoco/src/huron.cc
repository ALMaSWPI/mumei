# include "huron_mujoco/huron.h"

namespace huron {
namespace mujoco {

void Huron::Initialize() {
}

void Huron::SetUp() {
}

void Huron::Terminate() {
}

bool Huron::Move(const std::vector<double>& values) {
  return true;
}

bool Huron::Stop() {
//  return Move(std::vector<double>(12));
return true;
}

void BuildFromXml(const char* xml_path) {

  // Load model
  char error[1000] = "Could not load binary model";
  if (std::strlen(xml_path)>4 && !std::strcmp(argv[1]+std::strlen(argv[1])-4, ".mjb")) {
    this->m_ = mj_loadModel(xml_path[1], 0);
  } else {
    this->m_ = mj_loadXML(xml_path, 0, error, 1000);
  }
  if (!this->m_) {
    mju_error("Load model error: %s", error);
  }

  // Load data
  this->d_ = mj_makeData(m);
}

}  // namespace ros2
}  // namespace huron

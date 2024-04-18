# include "huron_mujoco/huron.h"

namespace huron {
namespace mujoco {

void Huron::Initialize() {
}

void Huron::SetUp() {
}

void Huron::Terminate() {
}

bool Huron::Move(const std::vector<double> values) {
  //Passing the joints to get the ids
  for(int i =0; i < joint_list_.size(); i ++){
    int id = mj_name2id(m_, mjOBJ_ACTUATOR, joint_list_[i]);
    if(id < 0){
      //joint name pass in is wrong
      return false;
    }
    d_->ctrl[id] = values[i];
  }

  return true;
}

bool Huron::Stop() {
  return Move(std::vector<double>(joint_list_.size(),0.0));
}

void Huron::BuildFromXml(char* xml_path, std::vector<char*> joint_list) {

  joint_list_ = joint_list;
  // Load model
  char error[1000] = "Could not load binary model";
  if (std::strlen(xml_path)>4) {
    m_ = mj_loadModel(xml_path, 0);
  } else {
    m_ = mj_loadXML(xml_path, 0, error, 1000);
  }
  if (!m_) {
    mju_error("Load model error: %s", error);
  }
  // Load data
  d_ = mj_makeData(m_);
}

void Huron::Loop(){
  mjtNum simstart = d_->time;
  while (d_->time - simstart < 1.0/60.0) {
    mj_step(m_, d_);
  }
}

void Huron::Controller(const mjModel* m_, mjData* d_){};


}  // namespace ros2
}  // namespace huron

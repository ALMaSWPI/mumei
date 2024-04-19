#pragma once

#include <functional>

namespace huron {

class Environment {
 public:
  Environment(std::function<void(void)> loop_func,
              std::function<void(void)> exit_func = []() {});
  Environment(const Environment&) = delete;
  Environment& operator=(const Environment&) = delete;
  virtual ~Environment() = default;

  virtual void Initialize(int argc, char* argv[]);
  virtual void Configure(void* config);
  virtual void Finalize();
  void Loop();
  virtual void Exit();

 protected:
  virtual void LoopPrologue();
  virtual void LoopEpilogue();
  
  std::function<void(void)> loop_func_;
  std::function<void(void)> exit_func_;
};

}  // namespace huron

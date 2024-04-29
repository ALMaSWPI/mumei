#include "mumei/environment.h"

namespace mumei {

Environment::Environment(std::function<void(void)> loop_func,
                         std::function<void(void)> exit_func)
    : loop_func_(loop_func), exit_func_(exit_func) {}

void Environment::Initialize(int argc, char* argv[]) {
}

void Environment::Configure(void* config) {
}

void Environment::Finalize() {
}

void Environment::Loop() {
  while (true) {
    LoopPrologue();
    loop_func_();
    LoopEpilogue();
  }
}

void Environment::Exit() {
  exit_func_();
}

void Environment::LoopPrologue() {}

void Environment::LoopEpilogue() {}

}  // namespace mumei

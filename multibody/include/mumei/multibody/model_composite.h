#pragma once

#include <vector>
#include <memory>

#include "model.h"

namespace mumei {
namespace multibody {

class ModelComposite final : public class Model {
 public:
  ModelComposite();

  void RegisterModel(std::unique_ptr<Model> model);

 private:
  std::vector<std::unique_ptr<Model>> models_;
};

}  // namespace multibody
}  // namespace mumei

#include "mumei/multibody/pinocchio_model_impl.h"
#include "mumei/exceptions/not_implemented_exception.h"

namespace mumei {
namespace multibody {
namespace internal {

struct PinocchioModelImpl::Impl {
  int dummy;
};

PinocchioModelImpl::PinocchioModelImpl()
    : ModelImplInterface() {
  throw NotImplementedException("Pinocchio not available!");
}
PinocchioModelImpl::~PinocchioModelImpl() = default;

}  // namespace internal
}  // namespace multibody
}  // namespace mumei

#include "huron/multibody/pinocchio_model_impl.h"
#include "huron/exceptions/not_implemented_exception.h"

namespace huron {
namespace multibody {
namespace internal {

template <typename T>
struct PinocchioModelImpl<T>::Impl {
  int dummy;
};

template <typename T>
PinocchioModelImpl<T>::PinocchioModelImpl()
    : ModelImplInterface<T>() {
  throw NotImplementedException("Pinocchio not available!");
}
template <typename T>
PinocchioModelImpl<T>::~PinocchioModelImpl() = default;

}  // namespace internal
}  // namespace multibody
}  // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::multibody::internal::PinocchioModelImpl)
HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::multibody::internal::PinocchioModelImpl)

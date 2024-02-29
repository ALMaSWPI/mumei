#pragma once

/**
 * These macros are inspired by Drake.
 * Source: https://github.com/RobotLocomotion/drake/blob/master/common/default_scalars.h
 */

/**
  * Declares that template instantiations exist for huron's default scalars.
  * This should only be used in .h files, never in .cc files.
  */
#define HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(  \
    SomeType) \
extern template SomeType<double>;

/** 
  * Defines template instantiations for huron's default scalars.
  * This should only be used in .cc files, never in .h files.
  */
#define HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS( \
    SomeType) \
template SomeType<double>;

#if HURON_USE_CASADI==1

/** 
  * Defines template instantiations for huron's autodiff scalars.
  * Currently, Casadi is supported.
  * This should only be used in .cc files, never in .h files.
  */
#define HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS( \
    SomeType) \
template SomeType<casadi::SX>;

/**
  * Declares that template instantiations exist for huron's autodiff scalars.
  * Currently, Casadi is supported.
  * This should only be used in .h files, never in .cc files.
  */
#define HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(  \
    SomeType) \
extern template SomeType<casadi::SX>;

#else

/** 
  * Defines template instantiations for huron's autodiff scalars.
  * Currently, Casadi is supported.
  * This should only be used in .cc files, never in .h files.
  */
#define HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS( \
    SomeType)

/**
  * Declares that template instantiations exist for huron's autodiff scalars.
  * Currently, Casadi is supported.
  * This should only be used in .h files, never in .cc files.
  */
#define HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(  \
    SomeType)
#endif

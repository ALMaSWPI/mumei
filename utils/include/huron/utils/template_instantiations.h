#pragma once

/**
 * These macros are inspired by Drake.
 * Source: https://github.com/RobotLocomotion/drake/blob/master/common/default_scalars.h
 */

/** 
  * Defines template instantiations for huron's default scalars.
  * This should only be used in .cc files, never in .h files.
  */
#define HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS( \
    SomeType) \
template SomeType<double>;

/**
  * Declares that template instantiations exist for huron's default scalars.
  * This should only be used in .h files, never in .cc files.
  */
#define HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(  \
    SomeType) \
extern template SomeType<double>;

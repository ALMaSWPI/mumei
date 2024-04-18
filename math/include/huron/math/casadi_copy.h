/**
 * The functions contained in this file are from the Pinocchio project
 * (https://github.com/stack-of-tasks/pinocchio) with the following license:
 *
 * BSD 2-Clause License

 * Copyright (c) 2014-2023, CNRS 
 * Copyright (c) 2018-2023, INRIA
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Pinocchio project.
*/
#pragma once

#if HURON_ENABLE_AUTODIFF==1
#if HURON_USE_CASADI==1

#include <casadi/casadi.hpp>
#include <eigen3/Eigen/Core>

namespace huron {

template<typename MT, typename Scalar>
inline void copy(::casadi::Matrix<Scalar> const & src,
                 Eigen::MatrixBase<MT> & dst)
{
  Eigen::DenseIndex const m = src.size1();
  Eigen::DenseIndex const n = src.size2();
  
  dst.resize(m, n);
  
  for (Eigen::DenseIndex i = 0; i < m; ++i)
    for (Eigen::DenseIndex j = 0; j < n; ++j)
      dst(i, j) = src(i, j);
}


// Copy Eigen matrix to casadi matrix
template<typename MT, typename Scalar>
inline void copy(Eigen::MatrixBase<MT> const & src,
                 ::casadi::Matrix<Scalar> & dst)
{
  Eigen::DenseIndex const m = src.rows();
  Eigen::DenseIndex const n = src.cols();
  
  dst.resize(m, n);
  
  for (Eigen::DenseIndex i = 0; i < m; ++i)
    for (Eigen::DenseIndex j = 0; j < n; ++j)
      dst(i, j) = src(i, j);
}
} // namespace huron

#endif  // HURON_USE_CASADI
#endif  // HURON_ENABLE_AUTODIFF

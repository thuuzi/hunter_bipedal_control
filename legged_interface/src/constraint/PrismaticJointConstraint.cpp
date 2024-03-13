/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include "legged_interface/constraint/PrismaticJointConstraint.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2
{
namespace legged_robot
{
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

PrismaticJointConstraint::PrismaticJointConstraint()
  : StateInputConstraint(ConstraintOrder::Linear)
{
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool PrismaticJointConstraint::isActive(scalar_t time) const
{
  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PrismaticJointConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                       const PreComputation& preComp) const
{
  vector_t res = matrix_t::Zero(getNumConstraints(time), 1);
  res(0) = input(14)-input(16);
  res(1) = input(20)-input(22);
  return res;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation PrismaticJointConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                              const vector_t& input,
                                                                              const PreComputation& preComp) const
{
  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, input, preComp);
  approx.dfdx = matrix_t::Zero(getNumConstraints(time), state.size());
  approx.dfdu = matrix_t::Zero(getNumConstraints(time), input.size());
  approx.dfdx(0,14) =1;
  approx.dfdx(0,16) =-1; 
  approx.dfdx(1,20) =1;
  approx.dfdx(1,22) =-1; 
  return approx;
}

}  // namespace legged_robot
}  // namespace ocs2

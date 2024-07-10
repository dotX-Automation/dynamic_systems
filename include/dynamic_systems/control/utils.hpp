/**
 * Dynamic systems library base class.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 *
 * June 29, 2023
 */

/**
 * Copyright 2024 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef DYNAMIC_SYSTEMS_CONTROL__UTILS_HPP_
#define DYNAMIC_SYSTEMS_CONTROL__UTILS_HPP_

#include <dynamic_systems/visibility_control.h>

#include <polynomial_kit/polynomial.hpp>

#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

using namespace Eigen;
using namespace PolynomialKit;

namespace DynamicSystems
{
  namespace Control
  {
    enum DYNAMIC_SYSTEMS_PUBLIC CommonLTIType : unsigned int {
      ORDER_0,
      ORDER_1,
      ORDER_2
    };

    enum DYNAMIC_SYSTEMS_PUBLIC ButterworthType : unsigned int {
      LOW_PASS,
      HIGH_PASS,
      BAND_PASS,
      NOTCH
    };

    void DYNAMIC_SYSTEMS_PUBLIC common_lti(CommonLTIType type,
      std::vector<double> params,
      Polynomiald & num, Polynomiald & den);

    void DYNAMIC_SYSTEMS_PUBLIC butterworth(ButterworthType type, 
      unsigned int order, std::vector<double> omegas,
      Polynomiald & num, Polynomiald & den);

    void DYNAMIC_SYSTEMS_PUBLIC realization(
      const Polynomiald & num, const Polynomiald & den,
      MatrixXd & A, MatrixXd & B, MatrixXd & C, MatrixXd & D);

    void DYNAMIC_SYSTEMS_PUBLIC feedforward(const double pole,
      const Polynomiald & p_num, const Polynomiald & p_den,
      Polynomiald & ff_num, Polynomiald & ff_den);

    void DYNAMIC_SYSTEMS_PUBLIC discretization_zoh(
      double ts, unsigned int steps, 
      const MatrixXd & A, const MatrixXd & B, const MatrixXd & C, const MatrixXd & D,
      MatrixXd & A_zoh, MatrixXd & B_zoh, MatrixXd & C_zoh, MatrixXd & D_zoh); 
  }
}

#endif  // DYNAMIC_SYSTEMS_CONTROL__UTILS_HPP_
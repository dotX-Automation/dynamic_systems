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

#include <dynamic_systems/control/utils.hpp>

namespace DynamicSystems
{
  namespace Control
  {
    void common_lti(CommonLTIType type,
      std::vector<double> params,
      Polynomiald & num, Polynomiald & den) 
    {
      if(type == CommonLTIType::ORDER_0) {
        if(params.size() != 1) {
          std::invalid_argument("Wrong number of parameters.");
        }
        double k = params.at(0);
        num.regrade(0);
        den.regrade(0);
        num.set(0, k);
        den.set(0, 1.0);
      }
      else if (type == CommonLTIType::ORDER_1) {
        if(params.size() != 2) {
          std::invalid_argument("Wrong number of parameters.");
        }
        double k = params.at(0);
        double tau = params.at(1);
        num.regrade(0);
        den.regrade(1);
        num.set(0, k);
        den.set(0, 1.0);
        den.set(1, tau);
      }
      else if (type == CommonLTIType::ORDER_2) {
        if(params.size() != 3) {
          std::invalid_argument("Wrong number of parameters.");
        }
        double k = params.at(0);
        double omega = params.at(1);
        double zeta = params.at(2);
        num.regrade(0);
        den.regrade(2);
        num.set(0, k * omega * omega);
        den.set(0, omega * omega);
        den.set(1, 2.0 * zeta * omega);
        den.set(2, 1.0);
      } else {
        std::invalid_argument("Unexpected type.");
      }
    }

    void butterworth(ButterworthType type, 
      unsigned int order, std::vector<double> omegas,
      Polynomiald & num, Polynomiald & den) 
    {
      if(type == ButterworthType::LOW_PASS || type == ButterworthType::HIGH_PASS) {
        if(omegas.size() != 1) {
          std::invalid_argument("Low/High pass filters need only one omega.");
        }
        double omega = omegas.at(0);
        unsigned int l = order/2;
        Polynomiald tmp;
        tmp.set(2, 1.0);
        den.regrade(0);
        den.set(0, 1.0);
        for(unsigned k = 0; k < l; k++) {
          double exps = (double(2*(k+1) + order - 1) / double(2 * order)) * M_PI;
          double re = std::cos(exps);
          double im = std::sin(exps);
          tmp.set(0, (omega*omega) * (re*re + im*im));
          tmp.set(1, -2.0 * omega * re);
          den *= tmp;
        }
        if(order%2 == 1) {
          tmp.regrade(1);
          tmp.set(0, omega);
          tmp.set(1, 1.0);
          den *= tmp;
        }
        if(type == ButterworthType::HIGH_PASS) {
          num.regrade(order);
          num.set(order, 1.0);
        } else {
          num.regrade(0);
          num = std::pow(omega, order);
        }
      }
      else if (type == ButterworthType::BAND_PASS) {
        if(omegas.size() != 2) {
          std::invalid_argument("Band pass filters need two omegas.");
        }
        Polynomiald num1, num2;
        Polynomiald den1, den2;
        butterworth(ButterworthType::HIGH_PASS, order, {omegas.at(0)}, num1, den1);
        butterworth(ButterworthType::LOW_PASS , order, {omegas.at(1)}, num2, den2);
        num = num1 * num2;
        den = den1 * den2;
      }
      else if (type == ButterworthType::NOTCH) {
        double omega = omegas.at(0);
        butterworth(ButterworthType::LOW_PASS , 2*order, {omegas.at(0)}, num, den);
        num.regrade(0);
        num.set(0, 1.0);
        Polynomiald tmp;
        tmp.set(0, 1.0);
        tmp.set(2, omega*omega);
        for(unsigned k = 0; k < order; k++) {
          num *= tmp;
        }
      }
      else {
        std::invalid_argument("Unexpected type.");
      }
    }

    void realization(
      const Polynomiald & num, const Polynomiald & den,
      MatrixXd & A, MatrixXd & B, MatrixXd & C, MatrixXd & D) 
    {
      unsigned int na = den.size();
      unsigned int nb = num.size();

      if(na < nb) {
        std::invalid_argument("The system is not realizable.");
      }

      unsigned int n = na - 1;
      A = MatrixXd::Zero(n, n);
      B = MatrixXd::Zero(n, 1);
      C = MatrixXd::Zero(1, n);
      D = MatrixXd::Zero(1, 1);

      if(n == 0) {
        D(0,0) = num.get(num.degree()) / den.get(den.degree());
      } else {
        for(unsigned int i = 0; i < n-1; i++) {
          A(i, i+1) = 1;
        }

        for(unsigned int i = 0; i < n; i++) {
          A(n-1, i) = -den.get(i) / den.get(den.degree());
        }
        B(n-1, 0) = 1;

        if(nb < na) {
          for(unsigned int i = 0; i < nb; i++) {
            C(0, i) = num.get(i) / den.get(den.degree());
          }
          D(0, 0) = 0;
        } else {
          for(unsigned int i = 0; i < n; i++) {
            C(0, i) = (num.get(i) / num.get(num.degree()) - den.get(i) / den.get(den.degree())) * num.get(num.degree()) / den.get(den.degree());
          }
          D(0, 0) = num.get(num.degree()) / den.get(den.degree());
        }
      }
    }

    void feedforward(const double pole,
      const Polynomiald & p_num, const Polynomiald & p_den,
      Polynomiald & ff_num, Polynomiald & ff_den)
    {
      unsigned int da = p_den.degree();
      unsigned int db = p_num.degree();

      if(da < db) {
        std::invalid_argument("The system is not realizable.");
      }

      Polynomiald poly;
      poly.set(0, 1);
      poly.set(1, 1/pole);

      ff_num = p_den;
      ff_den = p_num * (poly^(da-db));
    }

    void discretization_zoh(
      double time_sampling, unsigned int zoh_steps, 
      const MatrixXd & A, const MatrixXd & B, const MatrixXd & C, const MatrixXd & D,
      MatrixXd & A_zoh, MatrixXd & B_zoh, MatrixXd & C_zoh, MatrixXd & D_zoh)
    {
      if(time_sampling <= 0.0) {
        throw std::invalid_argument("Invalid time sampling.");
      }

      if(zoh_steps == 0) {
        throw std::invalid_argument("Invalid zoh steps.");
      }

      double dt = time_sampling / double(zoh_steps);
      MatrixXd H = MatrixXd::Zero(A.rows(), A.cols());
      MatrixXd H_prev = MatrixXd::Identity(A.rows(), A.cols());
      MatrixXd H_next = MatrixXd::Zero(A.rows(), A.cols());
      for(unsigned int i = 1; i <= zoh_steps; i++) {
        double t = i*dt;
        H_next = (A*t).exp();
        H += (0.5 * (H_prev + H_next)) * dt;
        H_prev = H_next;
      }

      A_zoh = (A*time_sampling).exp();
      B_zoh = H*B;
      C_zoh = C;
      D_zoh = D;
    }
  }
}

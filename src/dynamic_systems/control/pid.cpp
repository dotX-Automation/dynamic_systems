/**
 * Dynamic systems library base class.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 *
 * July 5, 2023
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

#include <dynamic_systems/control/pid.hpp>

namespace DynamicSystems
{
  namespace Control
  {
    /* InitParams */
    PIDInitParams::~PIDInitParams() {}

    std::unique_ptr<InitParams<double>> PIDInitParams::clone() const {
      std::unique_ptr<PIDInitParams> res = std::make_unique<PIDInitParams>();
      res->copy(*this);
      return res;
    }

    void PIDInitParams::copy(const InitParams<double> &other) {
      InitParams<double>::copy(other);
      auto &casted = static_cast<const PIDInitParams&>(other);
      this->time_sampling = casted.time_sampling;
      this->error_deadzone = casted.error_deadzone;
      this->integral_saturation = casted.integral_saturation;
      this->integral_reset_error_threshold = casted.integral_reset_error_threshold;
      this->integral_reset_divider = casted.integral_reset_divider;
      this->integral_reset_value = casted.integral_reset_value;
      this->derivative_filter_pole = casted.derivative_filter_pole;
      this->bumpless = casted.bumpless;
    }

    /* SetupParams */
    PIDSetupParams::~PIDSetupParams() {}

    std::unique_ptr<SetupParams<double>> PIDSetupParams::clone() const {
      std::unique_ptr<PIDSetupParams> res = std::make_unique<PIDSetupParams>();
      res->copy(*this);
      return res;
    }

    void PIDSetupParams::copy(const SetupParams<double> &other) {
      SetupParams<double>::copy(other);
      auto &casted = static_cast<const PIDSetupParams&>(other);
      this->k_p = casted.k_p;
      this->k_i = casted.k_i;
      this->k_d = casted.k_d;
    }

    /* State */
    PIDState::~PIDState() {}

    std::unique_ptr<State<double>> PIDState::clone() const {
      std::unique_ptr<PIDState> res = std::make_unique<PIDState>();
      res->copy(*this);
      return res;
    }

    void PIDState::copy(const State<double> &other) {
      State<double>::copy(other);
      auto &casted = static_cast<const PIDState&>(other);
      this->x_i = casted.x_i;
      this->x_d = casted.x_d;
    }

    /* System */
    void PIDSystem::init_parse(const InitParams<double>& initParams) {
      auto &casted = static_cast<const PIDInitParams&>(initParams);

      if(casted.time_sampling < 0.0) {
        throw std::invalid_argument("Invalid time sampling for pid system.");
      }
      if(casted.error_deadzone < 0.0) {
        throw std::invalid_argument("Invalid error deadzone for pid system.");
      }
      if(casted.integral_saturation < 0.0) {
        throw std::invalid_argument("Invalid integral saturation for pid system.");
      }
      if(casted.integral_reset_error_threshold < 0.0) {
        throw std::invalid_argument("Invalid integral reset error threshold for pid system.");
      }
      if(casted.integral_reset_error_threshold > 0.0 && casted.integral_reset_divider < 1.0) {
        throw std::invalid_argument("Invalid integral reset divider pid system.");
      }
      if(casted.integral_reset_error_threshold > 0.0 && casted.integral_reset_value < 0.0) {
        throw std::invalid_argument("Invalid integral reset value pid system.");
      }
      if(casted.derivative_filter_pole < 0.0) {
        throw std::invalid_argument("Invalid derivative filter der_pole_ for pid system.");
      }

      ts_ = casted.time_sampling;
      err_deadzone_ = casted.error_deadzone;
      int_sat_ = casted.integral_saturation;
      int_reset_thr_ = casted.integral_reset_error_threshold;
      int_reset_div_ = casted.integral_reset_divider;
      int_reset_val_ = casted.integral_reset_value;
      der_pole_ = casted.derivative_filter_pole;
      bumpless_ = casted.bumpless;

      if(ts_ > 0.0) {
        if(der_pole_ > 0.0) {
          A_ = std::exp(-der_pole_*ts_);
          B_ = (1.0-A_)/der_pole_;
          C_ = -der_pole_*der_pole_;
          D_ = der_pole_;
        } else {
          A_ = 0.0;
          B_ = 1.0;
          C_ = -1.0/ts_;
          D_ = +1.0/ts_;
        }
      } else {
        ts_ = 1.0;
        if(der_pole_ > 0.0) {
          A_ = std::exp(-der_pole_);
          B_ = 1.0-A_;
          C_ = -B_;
          D_ = +B_;
        } else {
          A_ = 0.0;
          B_ = 1.0;
          C_ = -B_;
          D_ = +B_;
        }
      }

      std::shared_ptr<PIDState> state = std::make_shared<PIDState>();
      state->x_i = 0.0;
      state->x_d = 0.0;

      reset(state);
      input(0.0);
      update();
    }

    void PIDSystem::setup_parse(const SetupParams<double>& setupParams) {
      auto &casted = static_cast<const PIDSetupParams&>(setupParams);
      kp_ = casted.k_p;
      ki_ = casted.k_i;
      kd_ = casted.k_d;
    }

    void PIDSystem::setup_default() {
      kp_ = 0.0;
      ki_ = 0.0;
      kd_ = 0.0;
    }

    void PIDSystem::deinit(){}

    void PIDSystem::state_validator(State<double> &state) {
      auto &state_casted = static_cast<PIDState&>(state);
      if(int_sat_ > 0.0 && std::abs(state_casted.x_i) > int_sat_) {
        state_casted.x_i = (state_casted.x_i < 0.0) ? -int_sat_ : int_sat_;
      }
    }

    void PIDSystem::input_validator(const State<double> &state, MatrixX<double> &input) {
      UNUSED(state);
      if(input.rows() != 1 || input.cols() != 1) {
        throw std::invalid_argument("Invalid input size.");
      }
    }

    void PIDSystem::dynamic_map(const State<double> &state, const MatrixX<double> &input, State<double> &next) {
      auto &state_casted = static_cast<const PIDState&>(state);
      auto &next_casted = static_cast<PIDState&>(next);

      double e = (std::abs(input(0,0)) < err_deadzone_) ? 0.0 : input(0,0);

      next_casted.x_i = state_casted.x_i + (bumpless_ ? ki_ * ts_ * e : ts_ * e);
      next_casted.x_d = A_ * state_casted.x_d + (bumpless_ ? kd_ * B_ * e : B_ * e);

      if(std::abs(e) > int_reset_thr_ && (e * next_casted.x_i) < 0.0) {
        next_casted.x_i /= int_reset_div_;
        if(std::abs(next_casted.x_i) > int_reset_val_) {
          next_casted.x_i = (next_casted.x_i < 0) ? -int_reset_val_ : int_reset_val_;
        }
      }
    }

    void PIDSystem::output_map(const State<double> &state, const MatrixX<double> &input, MatrixX<double>& output) {
      auto &state_casted = static_cast<const PIDState&>(state);
      output = MatrixX<double>(1,1);

      double e = (std::abs(input(0,0)) < err_deadzone_) ? 0.0 : input(0,0);

      if(bumpless_) {
        output(0,0) = (kp_ + kd_ * D_) * e + state_casted.x_i + C_ * state_casted.x_d;
      } else {
        output(0,0) = (kp_ + kd_ * D_) * e + ki_ * state_casted.x_i + kd_ * C_ * state_casted.x_d;
      }
    }
  }
}

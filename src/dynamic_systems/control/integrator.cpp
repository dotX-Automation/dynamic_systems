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

#include <dynamic_systems/control/integrator.hpp>

namespace DynamicSystems
{
  namespace Control
  {
    /* InitParams */
    IntegratorInitParams::~IntegratorInitParams() {}

    std::unique_ptr<InitParams<double>> IntegratorInitParams::clone() const {
      std::unique_ptr<IntegratorInitParams> res = std::make_unique<IntegratorInitParams>();
      res->copy(*this);
      return res;
    }

    void IntegratorInitParams::copy(const InitParams<double> &other) {
      InitParams<double>::copy(other);
      auto &casted = static_cast<const IntegratorInitParams&>(other);
      this->time_sampling = casted.time_sampling;
      this->rows = casted.rows;
      this->cols = casted.cols;
    }


    /* SetupParams */
    IntegratorSetupParams::~IntegratorSetupParams() {}

    std::unique_ptr<SetupParams<double>> IntegratorSetupParams::clone() const {
      std::unique_ptr<IntegratorSetupParams> res = std::make_unique<IntegratorSetupParams>();
      res->copy(*this);
      return res;
    }

    void IntegratorSetupParams::copy(const SetupParams<double> &other) {
      SetupParams<double>::copy(other);
      auto &casted = static_cast<const IntegratorSetupParams&>(other);
      this->multiplier = casted.multiplier;
      this->saturation = casted.saturation;
    }


    /* State */
    IntegratorState::~IntegratorState() {}

    std::unique_ptr<State<double>> IntegratorState::clone() const {
      std::unique_ptr<IntegratorState> res = std::make_unique<IntegratorState>();
      res->copy(*this);
      return res;
    }

    void IntegratorState::copy(const State<double> &other) {
      State<double>::copy(other);
      auto &casted = static_cast<const IntegratorState&>(other);
      this->value = casted.value;
    }

    /* System */
    void IntegratorSystem::init_parse(const InitParams<double>& initParams) {
      auto &casted = static_cast<const IntegratorInitParams&>(initParams);

      if(casted.time_sampling < 0.0) {
        throw std::invalid_argument("Invalid sampling time for integrator system.");
      }

      if(casted.time_sampling > 0.0) {
        this->ts_ = casted.time_sampling;
      } else {
        this->ts_ = 1.0;
      }

      std::shared_ptr<IntegratorState> state = std::make_shared<IntegratorState>();
      state->value = MatrixX<double>::Zero(casted.rows, casted.cols);

      reset(state);
      input(MatrixX<double>::Zero(casted.rows, casted.cols));
      update();
    }

    void IntegratorSystem::setup_parse(const SetupParams<double>& setupParams) {
      auto &casted = static_cast<const IntegratorSetupParams&>(setupParams);

      if(casted.saturation < 0.0) {
        throw std::invalid_argument("Invalid saturation for integrator system.");
      }

      this->mul_ = casted.multiplier;
      this->sat_ = casted.saturation;
    }

    void IntegratorSystem::setup_default() {
      this->mul_ = 1.0;
      this->sat_ = 1.0;
    }

    void IntegratorSystem::deinit(){}

    void IntegratorSystem::state_validator(State<double> &state) {
      auto &state_casted = static_cast<IntegratorState&>(state);
      if(this->sat_ > 0.0) {
        state_casted.value = state_casted.value.cwiseMax(-this->sat_).cwiseMin(this->sat_);
      }
    }

    void IntegratorSystem::input_validator(const State<double> &state, MatrixX<double> &input) {
      auto &state_casted = static_cast<const IntegratorState&>(state);
      if(input.rows() != state_casted.value.rows() || input.cols() != state_casted.value.cols()) {
        throw std::invalid_argument("Invalid input size.");
      }
    }

    void IntegratorSystem::dynamic_map(const State<double> &state, const MatrixX<double> &input, State<double> &next) {
      auto &state_casted = static_cast<const IntegratorState&>(state);
      auto &next_casted = static_cast<IntegratorState&>(next);
      next_casted.value = state_casted.value + this->mul_ * this->ts_ * input;
    }

    void IntegratorSystem::output_map(const State<double> &state, const MatrixX<double> &input, MatrixX<double>& output) {
      UNUSED(input);
      auto &state_casted = static_cast<const IntegratorState&>(state);
      output = state_casted.value;
    }
  }
}

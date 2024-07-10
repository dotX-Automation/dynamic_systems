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

#include <dynamic_systems/filters/jump_filter.hpp>

#include <iostream>

namespace DynamicSystems
{
  namespace Filters
  {
    /* InitParams */
    JumpFilterInitParams::~JumpFilterInitParams() {}

    std::unique_ptr<InitParams<double>> JumpFilterInitParams::clone() const {
      std::unique_ptr<JumpFilterInitParams> res = std::make_unique<JumpFilterInitParams>();
      res->copy(*this);
      return res;
    }

    void JumpFilterInitParams::copy(const InitParams<double> &other) {
      InitParams<double>::copy(other);
      auto casted = dynamic_cast<const JumpFilterInitParams&>(other);
      this->rows = casted.rows;
      this->cols = casted.cols;
    }

    /* SetupParams */
    JumpFilterSetupParams::~JumpFilterSetupParams() {}

    std::unique_ptr<SetupParams<double>> JumpFilterSetupParams::clone() const {
      std::unique_ptr<JumpFilterSetupParams> res = std::make_unique<JumpFilterSetupParams>();
      res->copy(*this);
      return res;
    }

    void JumpFilterSetupParams::copy(const SetupParams<double> &other) {
      SetupParams<double>::copy(other);
      auto casted = dynamic_cast<const JumpFilterSetupParams&>(other);
      this->update_lambda = casted.update_lambda;
      this->jump_threshold = casted.jump_threshold;
      this->recovery_initial = casted.recovery_initial;
      this->recovery_increase = casted.recovery_increase;
    }

    /* State */
    JumpFilterState::~JumpFilterState() {}

    std::unique_ptr<State<double>> JumpFilterState::clone() const {
      std::unique_ptr<JumpFilterState> res = std::make_unique<JumpFilterState>();
      res->copy(*this);
      return res;
    }

    void JumpFilterState::copy(const State<double> &other) {
      State<double>::copy(other);
      auto casted = dynamic_cast<const JumpFilterState&>(other);
      this->value = casted.value;
      this->target = casted.target;
      this->history = casted.history;
      this->jumping = casted.jumping;
      this->recovery = casted.recovery;
    }

    /* System */
    void JumpFilterSystem::init_parse(const InitParams<double>& initParams) {
      auto casted = dynamic_cast<const JumpFilterInitParams&>(initParams);

      std::shared_ptr<JumpFilterState> state = std::make_shared<JumpFilterState>();
      state->value = MatrixX<double>::Zero(casted.rows, casted.cols);
      state->target = MatrixX<double>::Zero(casted.rows, casted.cols);
      state->history = MatrixX<double>::Zero(casted.rows, casted.cols);
      state->jumping = false;
      state->recovery = 0.0;

      reset(state);
      input(MatrixX<double>(casted.rows, casted.cols));
      update();
    }

    void JumpFilterSystem::setup_parse(const SetupParams<double>& setupParams) {
      auto casted = dynamic_cast<const JumpFilterSetupParams&>(setupParams);

      if(casted.update_lambda < 0.0 || casted.update_lambda > 1.0) {
        throw std::invalid_argument("Invalid update lambda for jump filter system.");
      }

      if(casted.jump_threshold < 0.0) {
        throw std::invalid_argument("Invalid jumping threshold for jump filter system.");
      }

      if(casted.recovery_initial < 0.0) {
        throw std::invalid_argument("Invalid recovery initial value for jump filter system.");
      }

      if(casted.recovery_increase < 0.0) {
        throw std::invalid_argument("Invalid recovery increase value for jump filter system.");
      }

      this->lambda_ = casted.update_lambda;
      this->jump_thr_ = casted.jump_threshold;
      this->rec_init_ = casted.recovery_initial;
      this->rec_incr_ = casted.recovery_increase;
    }

    void JumpFilterSystem::setup_default() {
      this->lambda_ = 1.0;
      this->jump_thr_ = 0.0;
      this->rec_init_ = 0.0;
      this->rec_incr_ = 0.0;
    }

    void JumpFilterSystem::deinit(){}

    void JumpFilterSystem::state_validator(State<double> &state) {
      JumpFilterState &state_casted = static_cast<JumpFilterState&>(state);
      if(state_casted.value.rows() != state_casted.history.rows() ||
         state_casted.value.rows() != state_casted.target.rows() ||
         state_casted.value.cols() != state_casted.history.cols() ||
         state_casted.value.cols() != state_casted.target.cols())
      {
        throw std::invalid_argument("Inconsistent state matrices sizes.");
      }
    }

    void JumpFilterSystem::input_validator(const State<double> &state, MatrixX<double> &input) {
      const JumpFilterState &state_casted = static_cast<const JumpFilterState&>(state);
      if(input.rows() != state_casted.value.rows() || 
         input.cols() != state_casted.value.cols()) 
      {
        throw std::invalid_argument("Invalid input size.");
      }
    }

    void JumpFilterSystem::dynamic_map(const State<double> &state, const MatrixX<double> &input, State<double> &next) {
      const JumpFilterState &state_casted = static_cast<const JumpFilterState&>(state);
      JumpFilterState &next_casted = static_cast<JumpFilterState&>(next);

      MatrixX<double> difference = input - state_casted.target;
      MatrixX<double> correction = input - state_casted.value;
      bool jumping = state_casted.jumping;
      double recovery = state_casted.recovery;

      double jnrm = (difference - state_casted.history).norm();
      double cnrm = correction.norm();
      double l = this->lambda_;
      if(jumping) {
        recovery += this->rec_incr_;
        if(cnrm > recovery) {
          correction *= (recovery / cnrm);
        } else {
          jumping = false;
          recovery = 0.0;
        }
      } else if (jnrm >= jump_thr_ && this->rec_init_ < cnrm) {
        l = 0.0;
        jumping = true;
        recovery = this->rec_init_;
        correction *= (recovery / cnrm);
      } else {
        recovery = 0.0;
      }

      next_casted.value = jumping ? (state_casted.value + correction) : input;
      next_casted.target = input;
      next_casted.history = l * difference + (1.0 - l) * state_casted.history;
      next_casted.jumping = jumping;
      next_casted.recovery = recovery;
    }

    void JumpFilterSystem::output_map(const State<double> &state, const MatrixX<double> &input, MatrixX<double>& output) {
      UNUSED(input);
      const JumpFilterState &state_casted = static_cast<const JumpFilterState&>(state);
      output = state_casted.value;
    }
  }
}

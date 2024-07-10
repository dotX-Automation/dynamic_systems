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

#include <dynamic_systems/base/system.hpp>

namespace DynamicSystems
{
  namespace Base
  {
    /* InitParams */
    template <typename T>
    InitParams<T>::~InitParams() {}

    template <typename T>
    std::unique_ptr<InitParams<T>> InitParams<T>::clone() const {
      return std::make_unique<InitParams<T>>();
    }

    template <typename T>
    void InitParams<T>::copy(const InitParams<T> &other) {
      UNUSED(other);
    }


    /* SetupParams */
    template <typename T>
    SetupParams<T>::~SetupParams() {}

    template <typename T>
    std::unique_ptr<SetupParams<T>> SetupParams<T>::clone() const {
      return std::make_unique<SetupParams<T>>();
    }

    template <typename T>
    void SetupParams<T>::copy(const SetupParams<T> &other) {
      UNUSED(other);
    }


    /* State */
    template <typename T>
    State<T>::~State() {}

    template <typename T>
    std::unique_ptr<State<T>> State<T>::clone() const {
      return std::make_unique<State<T>>();
    }

    template <typename T>
    void State<T>::copy(const State<T> &other) {
      UNUSED(other);
    }


    /* System */
    template <typename T>
    System<T>::System() {
      reset_ = std::make_unique<State<T>>();
      state_ = std::make_unique<State<T>>();
      setup_default();
    }

    template <typename T>
    System<T>::~System(){
      if(inited_) {
        fini();
      }
    }

    template <typename T>
    void System<T>::init(std::shared_ptr<InitParams<T>> initParams) {
      if(initParams == nullptr) {
        throw std::invalid_argument("Null init parameters.");
      }
      init_parse(*initParams.get());
      inited_ = true;
      dirty_ = true;
    }

    template <typename T>
    void System<T>::setup(std::shared_ptr<SetupParams<T>> setupParams) {
      if(setupParams == nullptr) {
        setup_default();
      } else {
        setup_parse(*setupParams.get());
      }
      dirty_ = true;
    }

    template <typename T>
    void System<T>::fini() {
      if(inited_) {
        deinit();
        inited_ = false;
        dirty_ = true;
      }
    }

    template <typename T>
    bool System<T>::initialized() {
      return inited_;
    }

    template <typename T>
    bool System<T>::dirty() {
      return dirty_;
    }

    template <typename T>
    void System<T>::reset(std::shared_ptr<State<T>> state) {
      if(state) {
        reset_ = state->clone();
      }
      state_ = reset_->clone();
      state_validator(*state_.get());
      dirty_ = true;
    }

    template <typename T>
    void System<T>::input(const T& in) {
      MatrixX<T> m_in(1,1);
      m_in(0,0) = in;
      input(m_in);
    }

    template <typename T>
    void System<T>::input(const MatrixX<T>& in){
      input_ = in;
      input_validator(*state_.get(), input_);
      dirty_ = true;
    }

    template <typename T>
    MatrixX<T> System<T>::output(){
      update();
      return output_;
    }

    template <typename T>
    void System<T>::step() {
      std::unique_ptr<State<T>> next = state_->clone();
      if(dirty_) {
        input_validator(*state_.get(), input_);
      }
      dynamic_map(*state_.get(), input_, *next.get());
      state_ = std::move(next);
      state_validator(*state_.get());
      dirty_ = true;
    }

    template <typename T>
    void System<T>::update() {
      if(dirty_) {
        input_validator(*state_.get(), input_);
        output_map(*state_.get(), input_, output_);
        dirty_ = false;
      }
    }

    template <typename T>
    MatrixX<T> System<T>::evolve(const T& in) {
      MatrixX<T> m_in(1,1);
      m_in(0,0) = in;
      return evolve(m_in);
    }

    template <typename T>
    MatrixX<T> System<T>::evolve(const MatrixX<T>& in){
      input(in);
      MatrixX<T> res = output();
      step();
      return res;
    }

    template <typename T>
    std::shared_ptr<State<T>> System<T>::state() {
      return state_->clone();
    }

    template <typename T>
    std::array<unsigned int, 2u> System<T>::input_size(){
      std::array<unsigned int, 2u> size;
      size[0] = input_.rows();
      size[1] = input_.cols();
      return size;
    }

    template <typename T>
    std::array<unsigned int, 2u> System<T>::output_size(){
      std::array<unsigned int, 2u> size;
      MatrixX<T> output_ = output();
      size[0] = output_.rows();
      size[1] = output_.cols();
      return size;
    }

    template <typename T>
    void System<T>::init_parse(const InitParams<T>& initParams) {
      UNUSED(initParams);
    }

    template <typename T>
    void System<T>::setup_parse(const SetupParams<T>& setupParams) {
      UNUSED(setupParams);
    }

    template <typename T>
    void System<T>::setup_default() {}

    template <typename T>
    void System<T>::deinit() {}

    template <typename T>
    void System<T>::state_validator(State<T> &state) {
      UNUSED(state);
    }

    template <typename T>
    void System<T>::input_validator(const State<T> &state, MatrixX<T> &input) {
      UNUSED(state);
      UNUSED(input);
    }

    template <typename T>
    void System<T>::dynamic_map(const State<T> &state, const MatrixX<T> &input, State<T> &next) {
      UNUSED(input);
      next.copy(state);
    }

    template <typename T>
    void System<T>::output_map(const State<T> &state, const MatrixX<T> &input, MatrixX<T>& output) {
      UNUSED(state);
      output = input;
    }

    template class InitParams<double>;
    template class InitParams<float>;
    template class InitParams<long>;
    template class InitParams<int>;
    template class InitParams<short>;
    template class InitParams<char>;
    template class InitParams<bool>;

    template class SetupParams<double>;
    template class SetupParams<float>;
    template class SetupParams<long>;
    template class SetupParams<int>;
    template class SetupParams<short>;
    template class SetupParams<char>;
    template class SetupParams<bool>;

    template class State<double>;
    template class State<float>;
    template class State<long>;
    template class State<int>;
    template class State<short>;
    template class State<char>;
    template class State<bool>;

    template class System<double>;
    template class System<float>;
    template class System<long>;
    template class System<int>;
    template class System<short>;
    template class System<char>;
    template class System<bool>;

    template class InitParams<std::complex<double>>;
    template class InitParams<std::complex<float>>;
    template class InitParams<std::complex<long>>;
    template class InitParams<std::complex<int>>;
    template class InitParams<std::complex<short>>;
    template class InitParams<std::complex<char>>;
    template class InitParams<std::complex<bool>>;

    template class SetupParams<std::complex<double>>;
    template class SetupParams<std::complex<float>>;
    template class SetupParams<std::complex<long>>;
    template class SetupParams<std::complex<int>>;
    template class SetupParams<std::complex<short>>;
    template class SetupParams<std::complex<char>>;
    template class SetupParams<std::complex<bool>>;

    template class State<std::complex<double>>;
    template class State<std::complex<float>>;
    template class State<std::complex<long>>;
    template class State<std::complex<int>>;
    template class State<std::complex<short>>;
    template class State<std::complex<char>>;
    template class State<std::complex<bool>>;

    template class System<std::complex<double>>;
    template class System<std::complex<float>>;
    template class System<std::complex<long>>;
    template class System<std::complex<int>>;
    template class System<std::complex<short>>;
    template class System<std::complex<char>>;
    template class System<std::complex<bool>>;
  }
}

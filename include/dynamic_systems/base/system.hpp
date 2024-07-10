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

#ifndef DYNAMIC_SYSTEMS__SYSTEM_HPP_
#define DYNAMIC_SYSTEMS__SYSTEM_HPP_

#include <dynamic_systems/visibility_control.h>

#include <cmath>
#include <memory>
#include <Eigen/Core>

#define UNUSED(arg) (void)(arg)

using namespace Eigen;

namespace DynamicSystems
{
  namespace Base
  {
    /**
     * Holds init parameters for a dynamic system. Made to be derived from.
     */
    template <typename T>
    struct DYNAMIC_SYSTEMS_PUBLIC InitParams
    {
      virtual ~InitParams();
      virtual std::unique_ptr<InitParams> clone() const;
      virtual void copy(const InitParams &other);
    };

    /**
     * Holds setup parameters for a dynamic system. Made to be derived from.
     */
    template <typename T>
    struct DYNAMIC_SYSTEMS_PUBLIC SetupParams
    {
      virtual ~SetupParams();
      virtual std::unique_ptr<SetupParams> clone() const;
      virtual void copy(const SetupParams &other);
    };

    /**
     * Holds internal state for a dynamic system. Made to be derived from.
     */
    template <typename T>
    struct DYNAMIC_SYSTEMS_PUBLIC State
    {
      virtual ~State();
      virtual std::unique_ptr<State> clone() const;
      virtual void copy(const State &other);
    };

    /**
     * Base class for dynamic systems.
     */
    template <typename T>
    class DYNAMIC_SYSTEMS_PUBLIC System
    {
    public:
      System();
      ~System();

      void init(std::shared_ptr<InitParams<T>> initParams);
      void setup(std::shared_ptr<SetupParams<T>> setupParams);
      void fini();

      bool initialized();
      bool dirty();
      void reset(std::shared_ptr<State<T>> state = nullptr);
      void input(const T& in);
      void input(const MatrixX<T>& in);
      MatrixX<T> output();
      void step();
      void update();
      MatrixX<T> evolve(const T& in);
      MatrixX<T> evolve(const MatrixX<T>& in);

      std::shared_ptr<State<T>> state();
      std::array<unsigned int, 2u> input_size();
      std::array<unsigned int, 2u> output_size();

    protected:
      virtual void DYNAMIC_SYSTEMS_LOCAL init_parse(const InitParams<T>& initParams);
      virtual void DYNAMIC_SYSTEMS_LOCAL setup_parse(const SetupParams<T>& setupParams);
      virtual void DYNAMIC_SYSTEMS_LOCAL setup_default();
      virtual void DYNAMIC_SYSTEMS_LOCAL deinit();
      virtual void DYNAMIC_SYSTEMS_LOCAL state_validator(State<T> &state);
      virtual void DYNAMIC_SYSTEMS_LOCAL input_validator(const State<T> &state, MatrixX<T> &input);
      virtual void DYNAMIC_SYSTEMS_LOCAL dynamic_map(const State<T> &state, const MatrixX<T> &input, State<T> &next);
      virtual void DYNAMIC_SYSTEMS_LOCAL output_map(const State<T> &state, const MatrixX<T> &input, MatrixX<T> &output);

    private:
      bool inited_ = false; 
      bool dirty_ = true;
      std::unique_ptr<State<T>> reset_;
      std::unique_ptr<State<T>> state_;
      MatrixX<T> input_;
      MatrixX<T> output_;
    };

    using InitParamsd = InitParams<double>;
    using InitParamsf = InitParams<float>;
    using InitParamsl = InitParams<long>;
    using InitParamsi = InitParams<int>;
    using InitParamss = InitParams<short>;
    using InitParamsc = InitParams<char>;
    using InitParamsb = InitParams<bool>;

    using SetupParamsd = SetupParams<double>;
    using SetupParamsf = SetupParams<float>;
    using SetupParamsl = SetupParams<long>;
    using SetupParamsi = SetupParams<int>;
    using SetupParamss = SetupParams<short>;
    using SetupParamsc = SetupParams<char>;
    using SetupParamsb = SetupParams<bool>;

    using Stated = State<double>;
    using Statef = State<float>;
    using Statel = State<long>;
    using Statei = State<int>;
    using States = State<short>;
    using Statec = State<char>;
    using Stateb = State<bool>;

    using Systemd = System<double>;
    using Systemf = System<float>;
    using Systeml = System<long>;
    using Systemi = System<int>;
    using Systems = System<short>;
    using Systemc = System<char>;
    using Systemb = System<bool>;

    using InitParamscd = InitParams<std::complex<double>>;
    using InitParamscf = InitParams<std::complex<float>>;
    using InitParamscl = InitParams<std::complex<long>>;
    using InitParamsci = InitParams<std::complex<int>>;
    using InitParamscs = InitParams<std::complex<short>>;
    using InitParamscc = InitParams<std::complex<char>>;
    using InitParamscb = InitParams<std::complex<bool>>;

    using SetupParamscd = SetupParams<std::complex<double>>;
    using SetupParamscf = SetupParams<std::complex<float>>;
    using SetupParamscl = SetupParams<std::complex<long>>;
    using SetupParamsci = SetupParams<std::complex<int>>;
    using SetupParamscs = SetupParams<std::complex<short>>;
    using SetupParamscc = SetupParams<std::complex<char>>;
    using SetupParamscb = SetupParams<std::complex<bool>>;

    using Statecd = State<std::complex<double>>;
    using Statecf = State<std::complex<float>>;
    using Statecl = State<std::complex<long>>;
    using Stateci = State<std::complex<int>>;
    using Statecs = State<std::complex<short>>;
    using Statecc = State<std::complex<char>>;
    using Statecb = State<std::complex<bool>>;

    using Systemcd = System<std::complex<double>>;
    using Systemcf = System<std::complex<float>>;
    using Systemcl = System<std::complex<long>>;
    using Systemci = System<std::complex<int>>;
    using Systemcs = System<std::complex<short>>;
    using Systemcc = System<std::complex<char>>;
    using Systemcb = System<std::complex<bool>>;
  }
}

#endif // DYNAMIC_SYSTEMS__SYSTEM_HPP_

# dynamic_systems

C++ library to implement modular dynamic systems. Requires `Eigen`.

## Contents

This is a C++ ROS2 package that contains a modular dynamic systems library.

### `base`

This is the base library for all dynamic systems.

`DynamicSystems::Base::InitParams` is a struct that contains the initialization parameters of the dynamic systems. It is used to initialize the dynamic systems, and should be derived to contain the parameters of the specific dynamic systems. By default, it is empty.

`DynamicSystems::Base::SetupParams` is a struct that contains the setup parameters of the dynamic systems. It is used to change the dynamic systems' behaviour, and should be derived to contain the parameters of the specific dynamic systems. By default, it is empty.

`DynamicSystems::Base::State` is a struct that contains the state of the dynamic systems. It is used to retain the whole state of the dynamic systems, and should be derived to contain the internal variables of the specific dynamic systems. By default, it is empty.

`DynamicSystems::Base::System` is a virtual class that implements the core logic of a dynamic system. Its public methods call the internal protected methods, which should be implemented to obtain the desired behaviour.

The specialization of the structures `InitParams`, `SetupParams`, and `State` shall provide deep copy functionality throught `copy` and `clone` methods.

The virtual class `System` offers the following public methods:

- `void DynamicSystems::Base::System::init(std::shared_ptr<InitParams> initParams)`: initializes the dynamic system with the given parameters.
- `void DynamicSystems::Base::System::setup(std::shared_ptr<SetupParams> setupParams)`: change the dynamic system behaviour with the given parameters.
- `bool DynamicSystems::Base::System::fini()`: finalizes the filter, freeing any allocated memory, etc..
- `bool DynamicSystems::Base::System::initialized()`: whether the system is initialized..
- `void DynamicSystems::Base::System::dirty()`: whether the output of the system must be updated..
- `bool DynamicSystems::Base::System::reset(std::shared_ptr<State> state = nullptr)`: resets the filter to last provided resetting state.
- `bool DynamicSystems::Base::System::input(MatrixXd in)`: insert input inside the system.
- `MatrixXd DynamicSystems::Base::System::output()`: get output from the system. If dirty, update the output of the system before return.
- `void DynamicSystems::Base::System::step()`: update system's internal state.
- `void DynamicSystems::Base::System::update()`: update the output of the system, if dirty.
- `MatrixXd DynamicSystems::Base::System::evolve(MatrixXd in)`: execute in order `input`, `output` and `step`.

This class also offers the following protected interface (to be specialized by derived dynamic systems):

- `void DynamicSystems::Base::System::init_parse(const InitParams& initParams)`: parse the given parameters and initialize the system.
- `void DynamicSystems::Base::System::setup_parse(const SetupParams& setupParams)`: parse the given parameters and set system's internal parameters.
- `void DynamicSystems::Base::System::setup_default()`: set default values for system's internal parameters configuration.
- `void DynamicSystems::Base::System::deinit()`: release internal structures and allocated memory used by the derived system.
- `void DynamicSystems::Base::System::state_validator(State &state)`: validate and eventually change system's internal status at every change.
- `void DynamicSystems::Base::System::input_validator(const State &state, MatrixXd &input)`: validate and eventually change the passed input eventually considering the internal state.
- `void DynamicSystems::Base::System::dynamic_map(const State &state, const MatrixXd &input, State &next)`: specify the dynamic behaviour of the specified system.
- `void DynamicSystems::Base::System::output_map(const State &state, const MatrixXd &input, MatrixXd &output)`: specify the output behaviour of the specified system.

### `control`

This library contains implementations of dynamic systems and control functions.

Currently, the following dynamic systems are available:

- `DynamicSystems::Control::IntegratorSystem`: implements a matricial integrator.
- `DynamicSystems::Control::LTISystem`: implements a LTI MIMO system.
- `DynamicSystems::Control::PIDSystem`: implements a PID controller system.

It offer the following functions:

- `DynamicSystems::Control::common_lti`: produces transfer function for zero, first and second order common linear systems.
- `DynamicSystems::Control::butterworth`: computes butterworth filter transfer function from desired cutting frequencies.
- `DynamicSystems::Control::realization`: computes state space realization from laplace transfer function.
- `DynamicSystems::Control::distretization_zoh`: computes state space ZOH discretization.

### `filters`

This library contains implementations of filters.

Currently, the following dynamic systems are available:

- `DynamicSystems::Filters::JumpFilter`: implements a jump filter to limit signal's slope.

---

## Copyright and License

Copyright 2024 dotX Automation s.r.l.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at <http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

See the License for the specific language governing permissions and limitations under the License.

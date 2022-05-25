/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
 *
*/

#ifndef IGNITION_GAZEBO_SYSTEMS_WAVEVISUAL_HH_
#define IGNITION_GAZEBO_SYSTEMS_WAVEVISUAL_HH_

#include <memory>

#include "ignition/gazebo/System.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class WaveVisualPrivate;

  /// \brief A plugin for setting shaders to a visual and its params
  ///
  /// Plugin parameters:
  ///
  /// <shader>
  ///   <vertex>   Path to wave vertex program
  ///   <fragment> Path to wave fragment program
  /// <wavefield>  Wavefield parameters - see Wavefield.hh
  ///
  class WaveVisual
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: WaveVisual();

    /// \brief Destructor
    public: ~WaveVisual() override;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    /// Documentation inherited
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<WaveVisualPrivate> dataPtr;
  };
  }
}
}
}

#endif

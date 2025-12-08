/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <functional>

#include "../include/ros_gz_example_gazebo/BasicSystem.hh" // Assumes your header is in a matching folder

namespace sambirion_gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(BasicSystem)

  // This function is called once when the plugin is loaded.
  void BasicSystem::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/)
  {
    // Store a pointer to the world
    this->world = _world;

    // Connect to the world update event. This function is called on every iteration.
    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&BasicSystem::OnUpdate, this));

    gzmsg << "BasicSystem plugin has been loaded." << std::endl;
  }

  // This function is called on every simulation iteration.
  void BasicSystem::OnUpdate()
  {
    // The original logic was to print every 1000 iterations
    if (!this->world->IsPaused() && this->world->Iterations() % 1000 == 0)
    {
      gzmsg << "BasicSystem::OnUpdate" << std::endl;
    }
  }
}
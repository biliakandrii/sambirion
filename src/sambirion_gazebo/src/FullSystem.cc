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

#include "../include/ros_gz_example_gazebo/FullSystem.hh" // Assumes your header is in a matching folder

namespace sambirion_gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(FullSystem)

  // The 'Load' function is the equivalent of 'Configure' in the modern API.
  void FullSystem::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // Store pointers to the world and the plugin's SDF element
    this->world = _world;
    this->sdf = _sdf;

    gzmsg << "FullSystem::Load (equivalent to Configure)" << std::endl;

    // Connect our OnUpdate method to the world's update event.
    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&FullSystem::OnUpdate, this));
  }

  // In Gazebo Classic, PreUpdate, Update, and PostUpdate are consolidated.
  // You can connect to different events like WorldUpdateEnd for post-update logic,
  // but most logic goes in a single callback connected to WorldUpdateBegin.
  void FullSystem::OnUpdate()
  {
    if (!this->world->IsPaused() && this->world->Iterations() % 1000 == 0)
    {
      gzmsg << "FullSystem::OnUpdate (replaces PreUpdate, Update, PostUpdate)" << std::endl;
    }
  }

  // This function is called when the simulation is reset.
  void FullSystem::Reset()
  {
    gzmsg << "FullSystem::Reset" << std::endl;
    // Add logic here to reset your plugin's internal state.
  }
}
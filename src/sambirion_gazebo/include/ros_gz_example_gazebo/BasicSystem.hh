#ifndef SAMBIRION_GAZEBO_BASICSYSTEM_HH_
#define SAMBIRION_GAZEBO_BASICSYSTEM_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

namespace sambirion_gazebo
{
  class BasicSystem : public gazebo::WorldPlugin
  {
    public:
      /// @brief Called when the plugin is loaded by the simulator.
      void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

    private:
      /// @brief Called on every simulation iteration.
      void OnUpdate();

      /// @brief Pointer to the Gazebo world.
      gazebo::physics::WorldPtr world;

      /// @brief Connection to the world's update event.
      gazebo::event::ConnectionPtr updateConnection;
  };
}
#endif
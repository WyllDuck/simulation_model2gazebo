#ifndef GAZEBO_INS_DEVICES_HH
#define GAZEBO_INS_DEVICES_HH

// ROS
#include <ros/ros.h>

// ROS Msgs
#include <geometry_msgs/Vector3Stamped.h>

// Includes
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <fstream>
#include <mutex>
#include <thread>
#include <chrono>
#include <thread>
#include <string>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Plugin.hh>

namespace gazebo
{

   class INS_devices : public ModelPlugin
   {

   public:
      INS_devices();

      virtual ~INS_devices();

      virtual void Reset();

      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

   private:
      void Update();

      bool isLoopTime(const common::Time &time, double &dt);

      void GetState();

      void publish_state_truth();

      boost::shared_ptr<ros::NodeHandle> nh_;

      ros::Publisher pub_state_truth_accel;
      ros::Publisher pub_state_truth_vel;
      ros::Publisher pub_state_truth_pos;

      std::mutex mutex;

      event::ConnectionPtr updateConnection;

      transport::NodePtr gznode;

      physics::ModelPtr model_;

      common::Time last_sim_time_;
      double dt_required_;

      Eigen::VectorXd accel, vel, pos;
   };

} // namespace gazebo

#endif // GAZEBO_INS_DEVICES
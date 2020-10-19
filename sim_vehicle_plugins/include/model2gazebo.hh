/*
 * Copyright (c) 2018 Authors:
 *   - Félix Martí Valverde <martivalverde@hotmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef MODEL_TO_GAZEBO_HH
#define MODEL_TO_GAZEBO_HH

// ROS
#include <ros/ros.h>

// ROS Msgs
#include <geometry_msgs/Vector3Stamped.h>

// Includes
#include <eigen3/Eigen/Dense>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <algorithm>
#include <fstream>
#include <mutex>
#include <thread>
#include <chrono>
#include <thread>
#include <string>

// Gazebo
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// PhysicsModel Simulation
//#include <drone_model.hh>

namespace gazebo
{

    class ModelToGazebo : public ModelPlugin
    {

    public:
        ModelToGazebo();

        virtual ~ModelToGazebo();

        virtual void Reset();

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    private:

        //DronePhysicsModel drone;

        void Update();

        bool isLoopTime(const common::Time &time, double &dt);

        void SetState();
        void GetState();

        void UpdateModel(const double &dt);

        void PublishStateTruth();

        void GetGlobal2LocalMatrix(Eigen::Matrix3d &R_);

        Eigen::VectorXd gaz_ace, gaz_vel, gaz_pos;
        Eigen::VectorXd new_ace, new_vel, new_pos;

        Eigen::VectorXd inputs;

        ros::Publisher pub_state_truth_ace;
        ros::Publisher pub_state_truth_vel;
        ros::Publisher pub_state_truth_pos;

        boost::shared_ptr<ros::NodeHandle> nh;

        std::mutex mutex;

        event::ConnectionPtr updateConnection;

        transport::NodePtr gznode;

        physics::ModelPtr model;

        common::Time last_sim_time;
        double dt_required;
    };

} // namespace gazebo

#endif // MODEL_TO_GAZEBO_HH
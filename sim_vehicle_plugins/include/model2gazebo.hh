/*
 * Copyright (c) 2020 Authors:
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

// TF
#include <tf/transform_broadcaster.h>

// ROS Msgs
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>

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
#define MODEL_NAME DronePhysicsModel
#include <drone_model.hh>

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
        MODEL_NAME vehicle_model;

        void BroadcastTF(Eigen::VectorXd &_position, common::Time _cur_time, string _parent, string _child);

        void Update();

        bool isLoopTime(const common::Time &time, double &dt);

        void SetState();
        void GetState();

        void PublishStateTruth();

        Eigen::VectorXd gaz_ace, gaz_vel, gaz_pos;
        Eigen::Vector3d force, torque;

        ros::Publisher pub_state_truth_lin_ace;
        ros::Publisher pub_state_truth_lin_vel;
        ros::Publisher pub_state_truth_lin_pos;

        ros::Publisher pub_state_truth_ang_ace;
        ros::Publisher pub_state_truth_ang_vel;
        ros::Publisher pub_state_truth_ang_pos;

        boost::shared_ptr<ros::NodeHandle> nh;

        std::mutex mutex;

        event::ConnectionPtr updateConnection;

        transport::NodePtr gznode;

        physics::ModelPtr model;

        physics::LinkPtr base_link;

        common::Time last_sim_time;
        double dt_required;
        
        /* ---------
            INPUTS
         --------- */
        ros::Subscriber sub1, sub2, sub3, sub4, sub5, sub6;
        ros::Subscriber sub7, sub8, sub9, sub10, sub11, sub12;

        void callback_cmd1(const std_msgs::Float32::ConstPtr &msg) { inputs[0] = msg->data; }
        void callback_cmd2(const std_msgs::Float32::ConstPtr &msg) { inputs[1] = msg->data; }
        void callback_cmd3(const std_msgs::Float32::ConstPtr &msg) { inputs[2] = msg->data; }
        void callback_cmd4(const std_msgs::Float32::ConstPtr &msg) { inputs[3] = msg->data; }
        void callback_cmd5(const std_msgs::Float32::ConstPtr &msg) { inputs[4] = msg->data; }
        void callback_cmd6(const std_msgs::Float32::ConstPtr &msg) { inputs[5] = msg->data; }

        void callback_cmd7(const std_msgs::Float32::ConstPtr &msg) { inputs[6] = msg->data; }
        void callback_cmd8(const std_msgs::Float32::ConstPtr &msg) { inputs[7] = msg->data; }
        void callback_cmd9(const std_msgs::Float32::ConstPtr &msg) { inputs[8] = msg->data; }
        void callback_cmd10(const std_msgs::Float32::ConstPtr &msg) { inputs[9] = msg->data; }
        void callback_cmd11(const std_msgs::Float32::ConstPtr &msg) { inputs[10] = msg->data; }
        void callback_cmd12(const std_msgs::Float32::ConstPtr &msg) { inputs[11] = msg->data; }

        Eigen::VectorXd inputs;

    };

} // namespace gazebo

#endif // MODEL_TO_GAZEBO_HH
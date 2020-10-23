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

#ifndef DRONE_HH
#define DRONE_HH

// ROS
#include <ros/ros.h>

// Includes
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <chrono>
#include "yaml-cpp/yaml.h"

// Gazebo
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

using namespace std;

struct Parameters {

    // Parameters
    double mass, L;

    Eigen::Vector3d gravity;

    Eigen::Matrix3d I, kd;

    double k, b;

    // Print Parameters
    void Print(){

        cout << "**********************" << endl;
        cout << "Parameters:\n\n";

        cout << "\tmass: " << mass << endl;
        cout << "\tL: " << L << endl;
        cout << "\tk: " << k << endl;
        cout << "\tb: " << b << endl;
        cout << "\tgravity:\n" << gravity << endl;
        cout << "\tI:\n" << I << endl;
        cout << "\tkd:\n" << kd << endl;

        cout << "**********************" << endl;
    }
};

class DronePhysicsModel
{

public:
    DronePhysicsModel();

    ~DronePhysicsModel();

    void Run(Eigen::VectorXd &_inputs, Eigen::Vector3d &force_, Eigen::Vector3d &torque_);

    void Init(gazebo::physics::ModelPtr &_parent, sdf::ElementPtr &_sdf, boost::shared_ptr<ros::NodeHandle> &nh);

    void LoadParameters();

    void UpdateCurrentState(const Eigen::VectorXd &_cur_ace, const Eigen::VectorXd &_cur_vel, const Eigen::VectorXd &_cur_pos);

    void GetTransformtionMatrices();

    gazebo::physics::ModelPtr model;

    sdf::ElementPtr sdf;

    Eigen::VectorXd cur_pos, cur_ace, cur_vel;

    Eigen::Matrix3d R_Global2Local, R_Local2Global;

    boost::shared_ptr<ros::NodeHandle> nh;

    Parameters params;

private:

    void Thrust(Eigen::VectorXd &_inputs, Eigen::Vector3d &T_);

    void Torques(Eigen::VectorXd &_inputs, Eigen::Vector3d &tau_);

    void Acceleration(Eigen::VectorXd &_inputs, Eigen::Vector3d &a_);

    void AngularAcceleration(Eigen::VectorXd &_inputs, Eigen::Vector3d &omegadot_);
};

#endif // DRONE_HH
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
    double mass, lf, lr, s;

    Eigen::Vector3d gravity;

    Eigen::Matrix3d I, kd;

    double k, b;

    inline double checkValidity (YAML::Node &_config, string _key){
        if (_config[_key]) {
            return _config[_key].as<double>();
        } else {
            ROS_ERROR("ModelToGazebo (Drone Model): Check in 'sim_vehicle' the /conf folder or the params filled called in the URDF (in /urdf forlder). Missing Parameter: %s", _key.c_str());
        }
        return 0;
    }

    // Print Parameters
    void Print(){

        cout << "**********************" << endl;
        cout << "Parameters:\n\n";

        cout << "gravity:\n" << gravity << endl;

        cout << "--- dimensions ---" << endl;

        cout << "lf: " << lf << endl;
        cout << "lr: " << lr << endl;
        cout << "s: " << s << endl;

        cout << "--- propellers ---" << endl;

        cout << "k: " << k << endl;
        cout << "b: " << b << endl;

        cout << "--- body ---" << endl;

        cout << "I:\n" << I << endl;
        cout << "mass: " << mass << endl;
        cout << "kd:\n" << kd << endl;

        cout << "**********************" << endl;
    }
};

class DronePhysicsModel
{

public:
    DronePhysicsModel();

    ~DronePhysicsModel();

    void Run(const Eigen::VectorXd &_all_inputs, Eigen::Vector3d &force_, Eigen::Vector3d &torque_);

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

    void GetThrust(const Eigen::VectorXd &_inputs, Eigen::Vector3d &Tr_);

    void GetTorques(const Eigen::VectorXd &_inputs, Eigen::Vector3d &T_);

    void GetForces(const Eigen::VectorXd &_inputs, Eigen::Vector3d &F_);
};

#endif // DRONE_HH
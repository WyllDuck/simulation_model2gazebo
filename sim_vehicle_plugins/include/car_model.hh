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

#ifndef CAR_HH
#define CAR_HH

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
    double mass, lf, lr;

    Eigen::Vector3d gravity;
    double rho;

    Eigen::Matrix3d I;

    double Cm0, Cm1;
    double C0, C1;

    double Cd_A;

    double Caf_low, Car_low;
    double Caf_high, Car_high;

    double max_delta;

    inline double checkValidity (YAML::Node &_config, string _key){
        if (_config[_key]) {
            return _config[_key].as<double>();
        } else {
            ROS_ERROR("ModelToGazebo (Car Model): Check in 'sim_vehicle' the /conf folder or the params filled called in the URDF (in /urdf forlder). Missing Parameter: %s", _key.c_str());
        }
        return 0;
    }

    // Print Parameters
    void Print(){

        cout << "**********************" << endl;
        cout << "Parameters:\n\n";

        cout << "gravity:\n" << gravity << endl;
        cout << "rho: " << rho << endl;

        cout << "--- dimensions ---" << endl;

        cout << "lf: " << lf << endl;
        cout << "lr: " << lr << endl;

        cout << "max_delta: " << max_delta << endl;

        cout << "--- others ---" << endl;

        cout << "Cm0: " << Cm0 << endl;
        cout << "Cm1: " << Cm1 << endl;
        
        cout << "C0: " << C0 << endl;
        cout << "C1: " << C1 << endl;

        cout << "Cd_A: " << Cd_A << endl;
        
        cout << "Caf_low: " << Caf_low << endl;
        cout << "Car_low: " << Car_low << endl;

        cout << "Caf_high: " << Caf_high << endl;
        cout << "Car_high: " << Car_high << endl;

        cout << "--- body ---" << endl;

        cout << "I:\n" << I << endl;
        cout << "mass: " << mass << endl;

        cout << "**********************" << endl;
    }
};

class CarPhysicsModel
{

public:
    CarPhysicsModel();

    ~CarPhysicsModel();

    void Run(const Eigen::VectorXd &_all_inputs, Eigen::Vector3d &lin_, Eigen::Vector3d &ang_, double dt = -1);

    void Init(gazebo::physics::ModelPtr &_parent, sdf::ElementPtr &_sdf, boost::shared_ptr<ros::NodeHandle> &nh);

    void LoadParameters();

    void UpdateCurrentState(const Eigen::VectorXd &_cur_ace, const Eigen::VectorXd &_cur_vel, const Eigen::VectorXd &_cur_pos);

    void GetTransformtionMatrices();

    void SelectMode();

    gazebo::physics::ModelPtr model;

    sdf::ElementPtr sdf;

    Eigen::VectorXd cur_pos, cur_ace, cur_vel;

    Eigen::Matrix3d R_Global2Local, R_Local2Global;
    Eigen::Matrix3d Rx, Ry, Rz; // Used during Global2Local matrix calculation

    boost::shared_ptr<ros::NodeHandle> nh;

    Parameters params;

    int8_t mode = 0;

private:

    void DynamicModel (const double &_delta, const double &_motor, Eigen::Vector3d &F_, Eigen::Vector3d &T_);

    void KinematicModel (const double &_delta, const double &_motor, Eigen::Vector3d &V_, Eigen::Vector3d &W_, const double &dt);
    
};

#endif // CAR_HH
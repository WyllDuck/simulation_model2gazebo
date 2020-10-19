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

#ifndef PHYSICS_MODEL_HH
#define PHYSICS_MODEL_HH

// ROS
#include <ros/ros.h>

// Includes
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <chrono>

// Gazebo
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

class PhysicsModel
{

public:
    PhysicsModel();

    ~PhysicsModel();

    virtual void Init(gazebo::physics::ModelPtr &_parent, sdf::ElementPtr &_sdf, boost::shared_ptr<ros::NodeHandle> &nh);

    virtual void Run(const Eigen::VectorXd &_inputs, Eigen::VectorXd &fut_ace_);

    void UpdateCurrentState(const Eigen::VectorXd &_cur_ace, const Eigen::VectorXd &_cur_vel, const Eigen::VectorXd &_cur_pos);

    gazebo::physics::ModelPtr model;

    void GetTransformtionMatrices();

    Eigen::VectorXd cur_pos, cur_ace, cur_vel;
    Eigen::VectorXd fut_pos, fut_ace, fut_vel;

    Eigen::Matrix3d R_Global2Local, R_Local2Global;

    boost::shared_ptr<ros::NodeHandle> nh;
};

#endif // PHYSICS_MODEL_HH
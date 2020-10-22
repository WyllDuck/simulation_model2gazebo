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

#include "drone_model.hh"

using namespace Eigen;

/* ------------------------
     INIT, DEL & RUN
------------------------ */

DronePhysicsModel::DronePhysicsModel()
{
}

DronePhysicsModel::~DronePhysicsModel()
{
}

/* ------------------------
      BASIC FUNCTIONS
------------------------ */

void DronePhysicsModel::Init(gazebo::physics::ModelPtr &_parent, sdf::ElementPtr &_sdf, boost::shared_ptr<ros::NodeHandle> &_nh)
{
    // Save Model
    this->model = _parent;
    
    // Current State Vectors
    this->cur_ace.resize(6);
    this->cur_vel.resize(6);
    this->cur_pos.resize(6);

    // Future State Vectors
    this->fut_ace.resize(6);
    this->fut_vel.resize(6);
    this->fut_pos.resize(6);

    this->nh = _nh;
}

void DronePhysicsModel::UpdateCurrentState(const Eigen::VectorXd &_cur_ace, const Eigen::VectorXd &_cur_vel, const Eigen::VectorXd &_cur_pos)
{
    this->cur_ace = _cur_ace;
    this->cur_vel = _cur_vel;
    this->cur_pos = _cur_pos;
}

void DronePhysicsModel::GetTransformtionMatrices()
{
    double c0 = cos(cur_pos[3]);
    double c1 = cos(cur_pos[4]);
    double c2 = cos(cur_pos[5]);

    double s0 = sin(cur_pos[3]);
    double s1 = sin(cur_pos[4]);
    double s2 = sin(cur_pos[5]);

    R_Global2Local << c0 * c1, c0 * s1 * s2 - s0 * c2, c0 * s1 * s2 + s0 * s2,
        s0 * c1, s0 * s1 * s2 + c0 * c2, s0 * s1 * c2 - c0 * s2,
        -s1, c1 * s2, c1 * c2;

    R_Local2Global = R_Global2Local.inverse();
    
}

void DronePhysicsModel::Run(VectorXd &_inputs, VectorXd &fut_ace_)
{

    // Compute linear and angular accelerations.
    /*
    Vector3d a, omegadot;
    Acceleration(_inputs, a);
    AngularAcceleration(_inputs, omegadot);
    */

    // Append values for model2gazebo
    fut_ace_[0] = _inputs[0] * 10;
    fut_ace_[1] = _inputs[1] * 10;
    fut_ace_[2] = _inputs[2] * 10;

    fut_ace_[3] = _inputs[3] * 10;
    fut_ace_[4] = _inputs[4] * 10;
    fut_ace_[5] = _inputs[5] * 10;
}

/* ------------------------
    AUXILIARY FUNCTIONS
------------------------ */
// NOTE: Inputs (_inputs) are values for ωi

// Compute thrust given current inputs and thrust coefficient.
void DronePhysicsModel::Thrust(VectorXd &_inputs, Vector3d &T_)
{
    T_ << 0,
        0,
        k * _inputs.sum();
}

// Compute torques, given current inputs, length, drag coefficient, and thrust coefficient.
void DronePhysicsModel::Torques(VectorXd &_inputs, Vector3d &tau_)
{
    tau_ << L * k * (_inputs[0] - _inputs[2]),
            L * k * (_inputs[1] - _inputs[3]),
            b * (_inputs[0] - _inputs[1] + _inputs[2] - _inputs[3]);
}

// Compute acceleration in the global reference frame
void DronePhysicsModel::Acceleration(VectorXd &_inputs, Vector3d &a_)
{
    Vector3d T;
    Thrust(_inputs, T);

    Vector3d vel(cur_vel[0], cur_vel[1], cur_vel[2]);
    Vector3d Fd = -kd * vel;

    a_ = R_Local2Global / m * (T + Fd) + gravity;
}

// Compute angular acceleration in the global reference frame
void DronePhysicsModel::AngularAcceleration(VectorXd &_inputs, Vector3d &omegadot_)
{
    Vector3d tau;
    Torques(_inputs, tau);

    Vector3d omega(cur_vel[3], cur_vel[4], cur_vel[5]);
    omegadot_ = I.inverse() * (tau - (omega.cross(I * omega)));

    omegadot_ = R_Local2Global * omegadot_;
}
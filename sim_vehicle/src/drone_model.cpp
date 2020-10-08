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

using namespace Eigen;

void DronePhysicsModel::Run(VectorXd &_inputs, VectorXd &fut_ace_)
{

    // Compute linear and angular accelerations.
    Vector3d a, omegadot;
    Acceleration(_inputs, a);
    AngularAcceleration(_inputs, omegadot);

    // Append values for model2gazebo
    fut_ace_[0] = a[0];
    fut_ace_[1] = a[1];
    fut_ace_[2] = a[2];

    fut_ace_[3] = omegadot[0];
    fut_ace_[4] = omegadot[1];
    fut_ace_[5] = omegadot[2];
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
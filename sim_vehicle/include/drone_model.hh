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

#ifndef MODEL_HH
#define MODEL_HH

// Includes
#include <model.hh>

class DronePhysicsModel : Model
    {

    public:

        DronePhysicsModel();

        virtual ~DronePhysicsModel();

        virtual void run(Eigen::VectorXd &_inputs,
                         Eigen::VectorXd &_ace1, Eigen::VectorXd &_vel1, Eigen::VectorXd &_pos1, 
                         Eigen::VectorXd &_ace2, Eigen::VectorXd &_vel2, Eigen::VectorXd &_pos2);

    private:

     m, g, k, kd
      I, L, b, k

        void thrust(Eigen::Vector3d &_inputs, Eigen::Vector3d &T_);

        void torques(Eigen::Vector3d &_inputs, Eigen::Vector3d &tau_);

        void acceleration(Eigen::Vector3d &_inputs, Eigen::Vector3d &_theta, Eigen::Vector3d &_xdot, Eigen::Vector3d &a_);

        void angularAcceleration(Eigen::Vector3d &_inputs, Eigen::Vector3d &_omega, Eigen::Vector3d &omegadot_);

        void thetadot2omega(Eigen::Vector3d &_thetadot, Eigen::Vector3d &_theta, Eigen::Vector3d &omega_);

        void rotation(Eigen::Vector3d &_theta, Eigen::Matrix3d &R_);
        
    };

#endif // MODEL__HH
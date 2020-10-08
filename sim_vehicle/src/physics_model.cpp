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

#include <physics_model.hh>

void PhysicsModel::Init(gazebo::physics::ModelPtr &_parent, sdf::ElementPtr &_sdf, boost::shared_ptr<ros::NodeHandle> &_nh)
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

void PhysicsModel::UpdateCurrentState(const Eigen::VectorXd &_cur_ace, const Eigen::VectorXd &_cur_vel, const Eigen::VectorXd &_cur_pos)
{
    this->cur_ace = _cur_ace;
    this->cur_vel = _cur_vel;
    this->cur_pos = _cur_pos;
}

void PhysicsModel::GetTransformtionMatrices()
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
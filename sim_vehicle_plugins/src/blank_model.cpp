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

#include "blank_model.hh"

using namespace Eigen;

/* ------------------------
        INIT, DEL
------------------------ */

BlankModel::BlankModel()
{
}

BlankModel::~BlankModel()
{
}

/* ------------------------
        LOAD & INIT
------------------------ */

void BlankModel::LoadParameters (){

    // Load Parameters From Model
    gazebo::physics::InertialPtr inertial = this->model->GetLink("base_link")->GetInertial();

    params.I.setZero();
    params.I(0, 0) = inertial->IXX();
    params.I(0, 1) = inertial->IXY();
    params.I(0, 2) = inertial->IXZ();

    params.I(1, 0) = inertial->IXY();
    params.I(1, 1) = inertial->IYY();
    params.I(1, 2) = inertial->IYZ();

    params.I(2, 0) = inertial->IXZ();
    params.I(2, 1) = inertial->IYZ();
    params.I(2, 2) = inertial->IZZ();

    params.mass = inertial->Mass();

    // Load Parameters From YAML
    // Get YAML File Direction
    string yaml_config;
    if (this->sdf->HasElement("yaml_config")) {
        yaml_config = this->sdf->Get<string>("yaml_config");
    } else {
        ROS_ERROR("Model2Gazebo: 'yaml_config' parameter does not exist, check launch file.");
        return;
    }

    // Open YAML File
    ROS_INFO("Model2Gazebo: Loading parameters from %s", yaml_config.c_str());
    YAML::Node config = YAML::LoadFile(yaml_config);
    
    // Set Parameters Values
    params.gravity.setZero();
    params.gravity[2] = - config["gravity"].as<double>(); // SIGN CHANGE

    params.kd.setZero();
    params.kd(0, 0) = config["kd"]["xx"].as<double>();
    params.kd(1, 1) = config["kd"]["yy"].as<double>();
    params.kd(2, 2) = config["kd"]["zz"].as<double>();

    // Finished Notification
    ROS_INFO("Model2Gazebo: Parameters loaded successfully.");
    params.Print();
}

void BlankModel::Init(gazebo::physics::ModelPtr &_parent, sdf::ElementPtr &_sdf, boost::shared_ptr<ros::NodeHandle> &_nh)
{
    // Save Model
    this->model = _parent;
    this->sdf = _sdf;
    
    // Current State Vectors
    this->cur_ace.resize(6);
    this->cur_ace.setZero();

    this->cur_vel.resize(6);
    this->cur_vel.setZero();
    
    this->cur_pos.resize(6);
    this->cur_pos.setZero();

    // NodeHandler
    this->nh = _nh;

    // Load Parameters
    params = Parameters();
    LoadParameters();
}

/* ------------------------
      BASIC FUNCTIONS
------------------------ */

void BlankModel::UpdateCurrentState(const Eigen::VectorXd &_cur_ace, const Eigen::VectorXd &_cur_vel, const Eigen::VectorXd &_cur_pos)
{
    this->cur_ace = _cur_ace;
    this->cur_vel = _cur_vel;
    this->cur_pos = _cur_pos;
}

void BlankModel::GetTransformtionMatrices()
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

void BlankModel::Run(VectorXd &_inputs, Vector3d &force_, Vector3d &torque_)
{

    // Append values for model2gazebo - Reference Frame GLOBAL
    force_[0] = (_inputs[6] - _inputs[8]);
    force_[1] = (_inputs[9] - _inputs[7]);
    force_[2] = (_inputs[11] - _inputs[10]);

    torque_[0] = (_inputs[5] - _inputs[4]);
    torque_[1] = (_inputs[0] - _inputs[1]);
    torque_[2] = (_inputs[2] - _inputs[3]);
}
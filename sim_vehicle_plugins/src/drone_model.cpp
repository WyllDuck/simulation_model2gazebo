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

#include "drone_model.hh"

using namespace Eigen;

/* ------------------------
        INIT, DEL
------------------------ */

DronePhysicsModel::DronePhysicsModel()
{
}

DronePhysicsModel::~DronePhysicsModel()
{
}

/* ------------------------
        LOAD & INIT
------------------------ */

void DronePhysicsModel::LoadParameters()
{

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
    if (this->sdf->HasElement("yaml_config"))
    {
        yaml_config = this->sdf->Get<string>("yaml_config");
    }
    else
    {
        ROS_ERROR("Model2Gazebo: 'yaml_config' parameter does not exist, check launch file.");
        return;
    }

    // Open YAML File
    ROS_INFO("Model2Gazebo: Loading parameters from %s", yaml_config.c_str());
    YAML::Node config = YAML::LoadFile(yaml_config);

    // Set Parameters Values
    params.gravity.setZero();
    params.gravity[2] = -params.checkValidity(config, "gravity"); // SIGN CHANGE

    params.lf = params.checkValidity(config, "lf");
    params.lr = params.checkValidity(config, "lr");
    params.s = params.checkValidity(config, "s");

    params.k = params.checkValidity(config, "k");
    params.b = params.checkValidity(config, "b");

    params.kd.setZero();
    if (config["kd"])
    {
        params.kd(0, 0) = config["kd"]["xx"].as<double>();
        params.kd(1, 1) = config["kd"]["yy"].as<double>();
        params.kd(2, 2) = config["kd"]["zz"].as<double>();
    }
    else
    {
        ROS_ERROR("ModelToGazebo (Drone Model): Check in 'sim_vehicle' the /conf folder or the params filled called in the URDF (in /urdf forlder). Missing Parameter: kd");
    }

    // Finished Notification
    ROS_INFO("Model2Gazebo: Parameters loaded successfully.");
    params.Print();
}

void DronePhysicsModel::Init(gazebo::physics::ModelPtr &_parent, sdf::ElementPtr &_sdf, boost::shared_ptr<ros::NodeHandle> &_nh)
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

    Rx  <<  1,  0,  0,
            0, c0, s0,
            0,-s0, c0;

    Ry  <<  c1, 0,  s1,
            0,  1,   0,
            -s1, 0, c1;
    
    Rz  <<  c2,  s2, 0,
            -s2, c2, 0,
            0,   0,  1;

    R_Global2Local = Rx * Ry * Rz;
    R_Local2Global = R_Global2Local.inverse();
}

void DronePhysicsModel::Run(const VectorXd &_all_inputs, Vector3d &force_, Vector3d &torque_)
{
    // Update Transformation Matrixes (Global2Local & Local2Global)
    GetTransformtionMatrices();
    
    // Only 4 motors
    VectorXd _inputs;
    _inputs.resize(4);

    // TODO: Check input upper limit  
    _inputs[0] = _all_inputs[0];
    _inputs[1] = _all_inputs[1];
    _inputs[2] = _all_inputs[2];
    _inputs[3] = _all_inputs[3];

    // Append values for model2gazebo - Reference Frame GLOBAL
    GetForces(_inputs, force_);
    GetTorques(_inputs, torque_);
}

/* ------------------------
    AUXILIARY FUNCTIONS
------------------------ */
// NOTE: Inputs (_inputs) are values for ωi² | Squared Values!

// Compute thrust given current inputs and thrust coefficient.
void DronePhysicsModel::GetThrust(const VectorXd &_inputs, Vector3d &Tr_)
{
    Tr_ << 0,
        0,
        params.k * _inputs.sum();
}

// Compute torques, given current inputs, length, drag coefficient, and thrust coefficient.
void DronePhysicsModel::GetTorques(const VectorXd &_inputs, Vector3d &T_)
{
    T_ << params.lr * params.k * (_inputs[1] + _inputs[2]) - params.lf * params.k * (_inputs[0] + _inputs[3]),
        params.s * params.k * (- _inputs[0] - _inputs[1] + _inputs[2] + _inputs[3]),
        params.b * (_inputs[0] - _inputs[1] + _inputs[2] - _inputs[3]);

    T_ = R_Local2Global * T_;
}

// Compute acceleration in the global reference frame
void DronePhysicsModel::GetForces(const VectorXd &_inputs, Vector3d &F_)
{
    Vector3d Tr;
    Tr.resize(3);

    GetThrust(_inputs, Tr);

    Vector3d vel(cur_vel[0], cur_vel[1], cur_vel[2]);
    Vector3d Fd = - params.kd * vel;

    F_ = R_Local2Global * Tr + Fd + params.gravity;
}
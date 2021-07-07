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

#include "car_model.hh"

using namespace Eigen;

/* ------------------------
        INIT, DEL
------------------------ */

CarPhysicsModel::CarPhysicsModel()
{
}

CarPhysicsModel::~CarPhysicsModel()
{
}

/* ------------------------
        LOAD & INIT
------------------------ */

void CarPhysicsModel::LoadParameters()
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
    params.gravity[2] = params.checkValidity(config, "gravity"); // SIGN CHANGE

    params.rho = params.checkValidity(config, "rho"); // air density

    params.lf = params.checkValidity(config, "lf");
    params.lr = params.checkValidity(config, "lr");

    params.Cm0 = params.checkValidity(config, "Cm0");
    params.Cm1 = params.checkValidity(config, "Cm1");

    params.C0 = params.checkValidity(config, "C0");
    params.C1 = params.checkValidity(config, "C1");

    params.Cd_A = params.checkValidity(config, "Cd_A");

    params.Caf_low = params.checkValidity(config, "Caf_low");
    params.Car_low = params.checkValidity(config, "Car_low");

    params.Caf_high = params.checkValidity(config, "Caf_high");
    params.Car_high = params.checkValidity(config, "Car_high");

    params.max_delta = params.checkValidity(config, "max_delta");

    // Finished Notification
    ROS_INFO("Model2Gazebo: Parameters loaded successfully.");
    params.Print();
}

void CarPhysicsModel::Init(gazebo::physics::ModelPtr &_parent, sdf::ElementPtr &_sdf, boost::shared_ptr<ros::NodeHandle> &_nh)
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

void CarPhysicsModel::UpdateCurrentState(const Eigen::VectorXd &_cur_ace, const Eigen::VectorXd &_cur_vel, const Eigen::VectorXd &_cur_pos)
{
    this->cur_ace = _cur_ace;
    this->cur_vel = _cur_vel;
    this->cur_pos = _cur_pos;
}

void CarPhysicsModel::GetTransformtionMatrices()
{
    double c0 = cos(cur_pos[3]);
    double c1 = cos(cur_pos[4]);
    double c2 = cos(cur_pos[5]);

    double s0 = sin(cur_pos[3]);
    double s1 = sin(cur_pos[4]);
    double s2 = sin(cur_pos[5]);

    Rx  <<  1,  0,  0,
            0, c0,  s0,
            0, -s0, c0;

    Ry  <<  c1, 0, -s1,
            0,  1, 0,
            s1, 0, c1;
    
    Rz  <<  c2,  s2, 0,
            -s2, c2, 0,
            0,   0,  1;

    R_Global2Local = Rx * Ry * Rz;
    R_Local2Global = R_Global2Local.inverse();
}


void CarPhysicsModel::SelectMode()
{
    int8_t previous_mode = mode;
    double v = sqrt(pow(cur_vel[0], 2) + pow(cur_vel[1], 2));

    if (v > 4.0)
    {
        mode = 1;
    } 
    else
    {
        mode = 0;
    }

    // Debug
    if (previous_mode != mode) {
        ROS_INFO("CarPhysicsModel: mode has changed to %s", ((mode == 1) ? "Dynamic" : "Kinematic"));
    }

}


void CarPhysicsModel::Run(const VectorXd &_all_inputs, Vector3d &lin_, Vector3d &ang_, double dt)
{
    // Update Transformation Matrixes (Global2Local & Local2Global)
    GetTransformtionMatrices();

    // Append values for model2gazebo - Reference Frame GLOBAL
    switch (mode)
    {
    case 1:
        DynamicModel( - _all_inputs[0] * params.max_delta, - _all_inputs[5] * 20, lin_, ang_);
        break;
    
    case 0:
        KinematicModel( - _all_inputs[0] * params.max_delta, - _all_inputs[5] * 20, lin_, ang_, dt);
        break;

    default:
        break;
    }
}

/* ------------------------
    AUXILIARY FUNCTIONS
------------------------ */

void CarPhysicsModel::DynamicModel (const double &_delta, const double &_motor, Vector3d &F_, Vector3d &T_)
{
    // State
    double vx, vy, omega;

    vx      = cur_vel[0];
    vy      = cur_vel[1];
    omega   = cur_vel[5];

    // double Ffx = 0; // NOTE: No motrice force in the front

    // Motrice Force Direction X Wheels
    double sign_vx = (vx > 0) ? 1 : ((vx < 0) ? -1 : 0);
    double Frx = (params.Cm0 - params.Cm1*abs(vx))*_motor - sign_vx*(params.C0*abs(vx) + params.C1 + (params.Cd_A * params.rho * pow(vx,2)) / 2);

    // Lateral Forces
    double alpha_f = _delta - atan((vy + params.lf*omega) / abs(vx));    
    double alpha_r = atan((vy - params.lr*omega) / abs(vx));

    double Ffy, Fry;
  
    Ffy =   2.0 * params.Caf_high * 5 * alpha_f;
    Fry = - 2.0 * params.Car_high * 5 * alpha_r;

    F_[0] = Frx - Ffy * sin(_delta) + params.mass*vy*omega;
    F_[1] = Fry + Ffy * cos(_delta) - params.mass*vx*omega;
    
    F_ = R_Local2Global * F_;

    F_[2] -= params.gravity[2] * params.mass;

    T_[0] = 0;
    T_[1] = 0;
    T_[2] = Ffy*params.lf*cos(_delta) - Fry*params.lr;

    T_ = R_Local2Global * T_;

}

void CarPhysicsModel::KinematicModel (const double &_delta, const double &_motor, Vector3d &V_, Vector3d &W_, const double &dt)
{
    // State
    double vx = cur_vel[0];
    double sign_vx = (vx > 0) ? 1 : ((vx < 0) ? -1 : 0);

    double v = sqrt(pow(cur_vel[0], 2) + pow(cur_vel[1], 2)) * sign_vx;

    double a = (1 / params.mass) * ((params.Cm0 - params.Cm1*abs(vx))*_motor - sign_vx*(params.C0*abs(vx) + params.C1 + (params.Cd_A * params.rho * pow(vx,2)) / 2));

    double beta = atan2(params.lr * tan(_delta), params.lf + params.lr);

    V_[0] = (v + a * dt) * cos(beta);
    V_[1] = (v + a * dt) * sin(beta);
    V_[2] = 0;
    
    V_ = R_Local2Global * V_;

    W_[0] = 0;
    W_[1] = 0;
    W_[2] = v * sin(beta) / params.lr;

    W_ = R_Local2Global * W_;   
   
}
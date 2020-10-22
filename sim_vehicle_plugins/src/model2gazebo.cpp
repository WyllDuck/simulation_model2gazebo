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

#include "model2gazebo.hh"

using namespace gazebo;
using namespace ignition;

/* ------------------------
     INIT, DEL & RESET
------------------------ */

ModelToGazebo::ModelToGazebo() : ModelPlugin()
{

    // Gazebo State Vectors
    gaz_ace.resize(6);
    gaz_ace.setZero();
    
    gaz_vel.resize(6);
    gaz_vel.setZero();
    
    gaz_pos.resize(6);
    gaz_pos.setZero();

    // Model State Vectors
    new_ace.resize(6);
    new_ace.setZero();

    new_vel.resize(6);
    new_vel.setZero();
    
    new_pos.resize(6);
    new_pos.setZero();

    // Inputs Vector
    inputs.resize(12);
    inputs.setZero();

}

ModelToGazebo::~ModelToGazebo()
{
    ROS_INFO("ModelToGazebo: Closing.");
}

void ModelToGazebo::Reset()
{
    ModelPlugin::Reset();
}

/* ------------------------
        LOAD & UPDATE
------------------------ */

void ModelToGazebo::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{

    if (!ros::isInitialized())
    {
        int argc = 0;
        char *argv = nullptr;
        ros::init(argc, &argv, _sdf->Get<std::string>("node_name"), ros::init_options::NoSigintHandler);
    }

    this->nh.reset(new ros::NodeHandle(_sdf->Get<std::string>("node_name")));
    this->model = _parent;

    this->gznode = transport::NodePtr(new transport::Node());
    this->gznode->Init();
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelToGazebo::Update, this));

    dt_required = 1.0 / _sdf->Get<double>("rate");

    // Deactivate Gravity
    this->model->SetGravityMode(false);

    vehicle_model = DronePhysicsModel();
    vehicle_model.Init(_parent, _sdf, this->nh);

    // Inputs Subscribers
    this->sub1 = nh->subscribe<std_msgs::Float32>("/command/1", 1, &ModelToGazebo::callback_cmd1, this);
    this->sub2 = nh->subscribe<std_msgs::Float32>("/command/2", 1, &ModelToGazebo::callback_cmd2, this);
    this->sub3 = nh->subscribe<std_msgs::Float32>("/command/3", 1, &ModelToGazebo::callback_cmd3, this);
    this->sub4 = nh->subscribe<std_msgs::Float32>("/command/4", 1, &ModelToGazebo::callback_cmd4, this);
    this->sub5 = nh->subscribe<std_msgs::Float32>("/command/5", 1, &ModelToGazebo::callback_cmd5, this);
    this->sub6 = nh->subscribe<std_msgs::Float32>("/command/6", 1, &ModelToGazebo::callback_cmd6, this);

    this->sub7 = nh->subscribe<std_msgs::Float32>("/command/7", 1, &ModelToGazebo::callback_cmd7, this);
    this->sub8 = nh->subscribe<std_msgs::Float32>("/command/8", 1, &ModelToGazebo::callback_cmd8, this);
    this->sub9 = nh->subscribe<std_msgs::Float32>("/command/9", 1, &ModelToGazebo::callback_cmd9, this);
    this->sub10 = nh->subscribe<std_msgs::Float32>("/command/10", 1, &ModelToGazebo::callback_cmd10, this);
    this->sub11 = nh->subscribe<std_msgs::Float32>("/command/11", 1, &ModelToGazebo::callback_cmd11, this);
    this->sub12 = nh->subscribe<std_msgs::Float32>("/command/12", 1, &ModelToGazebo::callback_cmd12, this);

    // State Truth Publishers
    this->pub_state_truth_lin_ace = nh->advertise<geometry_msgs::Vector3Stamped>("/truth/linear/acceleration", 1000);
    this->pub_state_truth_lin_vel = nh->advertise<geometry_msgs::Vector3Stamped>("/truth/linear/velocity", 1000);
    this->pub_state_truth_lin_pos = nh->advertise<geometry_msgs::Vector3Stamped>("/truth/linear/position", 1000);

    this->pub_state_truth_ang_ace = nh->advertise<geometry_msgs::Vector3Stamped>("/truth/angular/acceleration", 1000);
    this->pub_state_truth_ang_vel = nh->advertise<geometry_msgs::Vector3Stamped>("/truth/angular/velocity", 1000);
    this->pub_state_truth_ang_pos = nh->advertise<geometry_msgs::Vector3Stamped>("/truth/angular/position", 1000);

}

void ModelToGazebo::Update()
{

    std::lock_guard<std::mutex> lock(mutex);

    common::Time cur_time = this->model->GetWorld()->SimTime();
    double dt;

    if (!isLoopTime(cur_time, dt))
    {
        return;
    }

    // Tasks
    /*START*/
    GetState();
    PublishStateTruth();

    // Load new state in PhysicsModel attributes.
    vehicle_model.UpdateCurrentState(gaz_ace, gaz_vel, gaz_pos);

    // Find Next State
    vehicle_model.Run(inputs, new_ace);

    SetState();
    /*END*/

    last_sim_time = cur_time;
}

bool ModelToGazebo::isLoopTime(const common::Time &time, double &dt)
{

    dt = (time - last_sim_time).Double();
    if (dt < 0)
    {
        this->Reset();
        return false;
    }
    else if (dt >= dt_required)
    {
        return true;
    }
    return false;
}

/* ------------------------
      ROS PUBLISHERS
------------------------ */

void ModelToGazebo::PublishStateTruth()
{

    geometry_msgs::Vector3Stamped msg_lin_ace = geometry_msgs::Vector3Stamped();
    geometry_msgs::Vector3Stamped msg_lin_vel = geometry_msgs::Vector3Stamped();
    geometry_msgs::Vector3Stamped msg_lin_pos = geometry_msgs::Vector3Stamped();

    geometry_msgs::Vector3Stamped msg_ang_ace = geometry_msgs::Vector3Stamped();
    geometry_msgs::Vector3Stamped msg_ang_vel = geometry_msgs::Vector3Stamped();
    geometry_msgs::Vector3Stamped msg_ang_pos = geometry_msgs::Vector3Stamped();

    // Header
    msg_lin_ace.header.stamp = msg_lin_vel.header.stamp = msg_lin_pos.header.stamp = ros::Time::now();
    msg_lin_ace.header.frame_id = msg_lin_vel.header.frame_id = msg_lin_pos.header.frame_id = "global";

    msg_ang_ace.header.stamp = msg_ang_vel.header.stamp = msg_ang_pos.header.stamp = ros::Time::now();
    msg_ang_ace.header.frame_id = msg_ang_vel.header.frame_id = msg_ang_pos.header.frame_id = "global";

    /* -------------------- */
    // Acceleration Linear
    msg_lin_ace.vector.x = gaz_ace[0];
    msg_lin_ace.vector.y = gaz_ace[1];
    msg_lin_ace.vector.z = gaz_ace[2];

    // Velocity Linear
    msg_lin_vel.vector.x = gaz_vel[0];
    msg_lin_vel.vector.y = gaz_vel[1];
    msg_lin_vel.vector.z = gaz_vel[2];

    // Position Linear
    msg_lin_pos.vector.x = gaz_pos[0];
    msg_lin_pos.vector.y = gaz_pos[1];
    msg_lin_pos.vector.z = gaz_pos[2];

    /* -------------------- */
    // Acceleration Angular
    msg_ang_ace.vector.x = gaz_ace[3];
    msg_ang_ace.vector.y = gaz_ace[4];
    msg_ang_ace.vector.z = gaz_ace[5];

    // Velocity Angular
    msg_ang_vel.vector.x = gaz_vel[3];
    msg_ang_vel.vector.y = gaz_vel[4];
    msg_ang_vel.vector.z = gaz_vel[5];

    // Position Angular
    msg_ang_pos.vector.x = gaz_pos[3];
    msg_ang_pos.vector.y = gaz_pos[4];
    msg_ang_pos.vector.z = gaz_pos[5];

    /* -------------------- */
    // Publish
    pub_state_truth_lin_ace.publish(msg_lin_ace);
    pub_state_truth_lin_vel.publish(msg_lin_vel);
    pub_state_truth_lin_pos.publish(msg_lin_pos);

    pub_state_truth_ang_ace.publish(msg_ang_ace);
    pub_state_truth_ang_vel.publish(msg_ang_vel);
    pub_state_truth_ang_pos.publish(msg_ang_pos);
}

/* ------------------------
    INTERACT WITH GAZEBO
------------------------ */

void ModelToGazebo::SetState()
{
    
    // Acceleration
    this->model->GetLink("base_link")->SetForce(math::Vector3d(new_ace[0], new_ace[1], new_ace[2]));
    this->model->GetLink("base_link")->SetTorque(math::Vector3d(new_ace[3], new_ace[4], new_ace[5]));
}

void ModelToGazebo::GetState()
{

    /* ----------------------------------
            Reference Frame Global 
    ---------------------------------- */
    // position (x, y, z, roll, pitch, yaw)
    math::Pose3d global_pos_gaz = this->model->WorldPose();

    // mini append information
    gaz_pos[0] = global_pos_gaz.Pos()[0];
    gaz_pos[1] = global_pos_gaz.Pos()[1];
    gaz_pos[2] = global_pos_gaz.Pos()[2];

    gaz_pos[3] = global_pos_gaz.Rot().Roll();
    gaz_pos[4] = global_pos_gaz.Rot().Pitch();
    gaz_pos[5] = global_pos_gaz.Rot().Yaw();

    // velocity
    math::Vector3d lin_vel_gaz = this->model->WorldLinearVel();
    math::Vector3d ang_vel_gaz = this->model->WorldAngularVel();

    // acceleration
    math::Vector3d lin_accel_gaz = this->model->WorldLinearAccel();
    math::Vector3d ang_accel_gaz = this->model->WorldAngularAccel();

    /* ----------------------------------
        Reference Frame Local (Vehicle) 
    ---------------------------------- */
    // transformation tools
    Eigen::Matrix3d R;
    GetGlobal2LocalMatrix(R);

    // velocity
    Eigen::Vector3d lin_global_velocity(lin_vel_gaz[0], lin_vel_gaz[1], lin_vel_gaz[2]);
    Eigen::Vector3d lin_local_velocity = R * lin_global_velocity;

    Eigen::Vector3d ang_global_velocity(ang_vel_gaz[0], ang_vel_gaz[1], ang_vel_gaz[2]);
    Eigen::Vector3d ang_local_velocity = R * ang_global_velocity;

    // acceleration
    Eigen::Vector3d lin_global_acceleration(lin_accel_gaz[0], lin_accel_gaz[1], lin_accel_gaz[2]);
    Eigen::Vector3d lin_local_acceleration = R * lin_global_acceleration;

    Eigen::Vector3d ang_global_acceleration(ang_accel_gaz[0], ang_accel_gaz[1], ang_accel_gaz[2]);
    Eigen::Vector3d ang_local_acceleration = R * ang_global_acceleration;

    /* ----------------------------------
        Append Information 
    ---------------------------------- */
    gaz_ace[0] = lin_local_acceleration[0];
    gaz_ace[1] = lin_local_acceleration[1];
    gaz_ace[2] = lin_local_acceleration[2];

    gaz_ace[3] = ang_local_acceleration[0];
    gaz_ace[4] = ang_local_acceleration[1];
    gaz_ace[5] = ang_local_acceleration[2];

    gaz_vel[0] = lin_local_velocity[0];
    gaz_vel[1] = lin_local_velocity[1];
    gaz_vel[2] = lin_local_velocity[2];

    gaz_vel[3] = ang_local_velocity[0];
    gaz_vel[4] = ang_local_velocity[1];
    gaz_vel[5] = ang_local_velocity[2];
}

/* ------------------------
    AUXILIARY FUNCTIONS
------------------------ */

void ModelToGazebo::GetGlobal2LocalMatrix(Eigen::Matrix3d &R_)
{

    double c0 = cos(gaz_pos[3]);
    double c1 = cos(gaz_pos[4]);
    double c2 = cos(gaz_pos[5]);

    double s0 = sin(gaz_pos[3]);
    double s1 = sin(gaz_pos[4]);
    double s2 = sin(gaz_pos[5]);

    R_ << c0 * c1, c0 * s1 * s2 - s0 * c2, c0 * s1 * s2 + s0 * s2,
        s0 * c1, s0 * s1 * s2 + c0 * c2, s0 * s1 * c2 - c0 * s2,
        -s1, c1 * s2, c1 * c2;
}

GZ_REGISTER_MODEL_PLUGIN(ModelToGazebo)
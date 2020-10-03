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
    gaz_vel.resize(6);
    gaz_pos.resize(6);

    // Model State Vectors
    new_ace.resize(6);
    new_vel.resize(6);
    new_pos.resize(6);
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

    // Publishers State Truth
    this->pub_state_truth_ace = nh->advertise<geometry_msgs::Vector3Stamped>("/truth/acceleration", 1000);
    this->pub_state_truth_vel = nh->advertise<geometry_msgs::Vector3Stamped>("/truth/velocity", 1000);
    this->pub_state_truth_pos = nh->advertise<geometry_msgs::Vector3Stamped>("/truth/position", 1000);

    this->model = _parent;

    this->gznode = transport::NodePtr(new transport::Node());
    this->gznode->Init();
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelToGazebo::Update, this));

    dt_required = 1.0 / _sdf->Get<double>("rate");
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
    UpdateModel(dt);
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
      MODEL UPDATER
------------------------ */

void ModelToGazebo::UpdateModel(const double &dt)
{
    //new_ace = new_ace;
    new_vel = gaz_vel + dt * gaz_ace;
    new_pos = gaz_pos + dt * gaz_vel;
}

/* ------------------------
      ROS PUBLISHERS
------------------------ */

void ModelToGazebo::PublishStateTruth()
{

    geometry_msgs::Vector3Stamped msg_ace = geometry_msgs::Vector3Stamped();
    geometry_msgs::Vector3Stamped msg_vel = geometry_msgs::Vector3Stamped();
    geometry_msgs::Vector3Stamped msg_pos = geometry_msgs::Vector3Stamped();

    // Header
    msg_ace.header.stamp = msg_vel.header.stamp = msg_pos.header.stamp = ros::Time::now();
    msg_ace.header.frame_id = msg_vel.header.frame_id = msg_pos.header.frame_id = "global";

    // Acceleration
    msg_ace.vector.x = gaz_ace[0];
    msg_ace.vector.y = gaz_ace[1];
    msg_ace.vector.z = gaz_ace[2];

    // Velocity
    msg_vel.vector.x = gaz_vel[0];
    msg_vel.vector.y = gaz_vel[1];
    msg_vel.vector.z = gaz_vel[2];

    // Position
    msg_pos.vector.x = gaz_pos[0];
    msg_pos.vector.y = gaz_pos[1];
    msg_pos.vector.z = gaz_pos[2];

    // Publish
    pub_state_truth_ace.publish(msg_ace);
    pub_state_truth_vel.publish(msg_vel);
    pub_state_truth_pos.publish(msg_pos);
}

/* ------------------------
    INTERACT WITH GAZEBO
------------------------ */

void ModelToGazebo::SetState()
{
    /*
    // Acceleration
    this->model->SetLinearAccel(math::Vector3d(new_ace[0], new_ace[1], new_ace[2]));
    this->model->SetAngularAccel(math::Vector3d(new_ace[3], new_ace[4], new_ace[5]));
    */

    // Velocity
    this->model->SetLinearVel(math::Vector3d(new_vel[0], new_vel[1], new_vel[2]));
    this->model->SetAngularVel(math::Vector3d(new_vel[3], new_vel[4], new_vel[5]));

    // Position
    this->model->SetWorldPose(math::Pose3(new_pos[0], new_pos[1], new_pos[2], new_pos[3], new_pos[4], new_pos[5]));
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
    GetRotationMatrix(R);

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

void ModelToGazebo::GetRotationMatrix(Eigen::Matrix3d &R_)
{

    double c0 = cos(gaz_pos[3]);
    double c1 = cos(gaz_pos[4]);
    double c2 = cos(gaz_pos[5]);

    double s0 = sin(gaz_pos[3]);
    double s1 = sin(gaz_pos[4]);
    double s2 = sin(gaz_pos[5]);

    R_ << c0 * c2 - c1 * s0 * s2, -c2 * s0 - c0 * c1 * s2, s1 * s2,
        c1 * c2 * s0 + c0 * s2, c0 * c1 * c2 - s0 * s2, -c2 * s1,
        s0 * s1, c0 * s1, c1;
}

GZ_REGISTER_MODEL_PLUGIN(ModelToGazebo)
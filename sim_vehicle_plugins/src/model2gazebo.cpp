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
    
    gaz_vel_global.resize(6);
    gaz_vel_global.setZero();

    gaz_vel_local.resize(6);
    gaz_vel_local.setZero();
    
    gaz_pos.resize(6);
    gaz_pos.setZero();

    // Model State Vectors
    force.setZero();
    torque.setZero();
    
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
    this->base_link = this->model->GetLink("base_link");

    this->gznode = transport::NodePtr(new transport::Node());
    this->gznode->Init();
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelToGazebo::Update, this));

    dt_required = 1.0 / _sdf->Get<double>("rate");

    // Deactivate Gravity & Wind
    this->model->SetGravityMode(false);
    this->model->SetWindMode(false);

    vehicle_model = MODEL_NAME();
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
    this->pub_state_truth_lin_ace = nh->advertise<geometry_msgs::Vector3Stamped>("/truth/global/linear/acceleration", 1000);
    this->pub_state_truth_lin_vel_global = nh->advertise<geometry_msgs::Vector3Stamped>("/truth/global/linear/velocity", 1000);
    this->pub_state_truth_lin_vel_local = nh->advertise<geometry_msgs::Vector3Stamped>("/truth/local/linear/velocity", 1000);
    this->pub_state_truth_lin_pos = nh->advertise<geometry_msgs::Vector3Stamped>("/truth/global/linear/position", 1000);

    this->pub_state_truth_ang_ace = nh->advertise<geometry_msgs::Vector3Stamped>("/truth/global/angular/acceleration", 1000);
    this->pub_state_truth_lin_ang_global = nh->advertise<geometry_msgs::Vector3Stamped>("/truth/global/angular/velocity", 1000);
    this->pub_state_truth_lin_ang_local = nh->advertise<geometry_msgs::Vector3Stamped>("/truth/local/angular/velocity", 1000);
    this->pub_state_truth_ang_pos = nh->advertise<geometry_msgs::Vector3Stamped>("/truth/global/angular/position", 1000);

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
    vehicle_model.UpdateCurrentState(gaz_ace, gaz_vel_local, gaz_pos);

    // Find Next State
    vehicle_model.Run(inputs, force, torque);

    SetState();
    /*END*/

    BroadcastTF(gaz_pos, cur_time, "map", "base_link");
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

// TF Broadcaster Function
void ModelToGazebo::BroadcastTF(Eigen::VectorXd &_position, common::Time _cur_time, string _parent, string _child)
{
    // TF Broadcaster
    static tf::TransformBroadcaster br;

    // Publish TF
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(_position[0], _position[1], _position[2]) );
    
    tf::Quaternion q;
    q.setRPY(_position[3], _position[4], _position[5]);
    transform.setRotation(q);
    
    br.sendTransform(tf::StampedTransform(transform, ros::Time(_cur_time.Double()), _parent, _child));
}

void ModelToGazebo::PublishStateTruth()
{

    geometry_msgs::Vector3Stamped msg_lin_ace = geometry_msgs::Vector3Stamped();
    geometry_msgs::Vector3Stamped msg_lin_vel_global = geometry_msgs::Vector3Stamped();
    geometry_msgs::Vector3Stamped msg_lin_vel_local = geometry_msgs::Vector3Stamped();
    geometry_msgs::Vector3Stamped msg_lin_pos = geometry_msgs::Vector3Stamped();

    geometry_msgs::Vector3Stamped msg_ang_ace = geometry_msgs::Vector3Stamped();
    geometry_msgs::Vector3Stamped msg_ang_vel_global = geometry_msgs::Vector3Stamped();
    geometry_msgs::Vector3Stamped msg_ang_vel_local = geometry_msgs::Vector3Stamped();
    geometry_msgs::Vector3Stamped msg_ang_pos = geometry_msgs::Vector3Stamped();

    // Header
    msg_lin_ace.header.stamp = msg_lin_vel_global.header.stamp = msg_lin_pos.header.stamp = ros::Time::now();
    msg_lin_ace.header.frame_id = msg_lin_vel_global.header.frame_id = msg_lin_pos.header.frame_id = "map";
    msg_lin_vel_local.header.stamp = ros::Time::now();
    msg_lin_vel_local.header.frame_id = "base_link";

    msg_ang_ace.header.stamp = msg_ang_vel_global.header.stamp = msg_ang_pos.header.stamp = ros::Time::now();
    msg_ang_ace.header.frame_id = msg_ang_vel_global.header.frame_id = msg_ang_pos.header.frame_id = "map";
    msg_ang_vel_local.header.stamp = ros::Time::now();
    msg_ang_vel_local.header.frame_id = "base_link";

    /* -------------------- */
    // Acceleration Linear
    msg_lin_ace.vector.x = gaz_ace[0];
    msg_lin_ace.vector.y = gaz_ace[1];
    msg_lin_ace.vector.z = gaz_ace[2];

    // Velocity Linear Global
    msg_lin_vel_global.vector.x = gaz_vel_global[0];
    msg_lin_vel_global.vector.y = gaz_vel_global[1];
    msg_lin_vel_global.vector.z = gaz_vel_global[2];

    // Velocity Linear Local
    msg_lin_vel_local.vector.x = gaz_vel_local[0];
    msg_lin_vel_local.vector.y = gaz_vel_local[1];
    msg_lin_vel_local.vector.z = gaz_vel_local[2];

    // Position Linear
    msg_lin_pos.vector.x = gaz_pos[0];
    msg_lin_pos.vector.y = gaz_pos[1];
    msg_lin_pos.vector.z = gaz_pos[2];

    /* -------------------- */
    // Acceleration Angular
    msg_ang_ace.vector.x = gaz_ace[3];
    msg_ang_ace.vector.y = gaz_ace[4];
    msg_ang_ace.vector.z = gaz_ace[5];

    // Velocity Angular Global
    msg_ang_vel_global.vector.x = gaz_vel_global[3];
    msg_ang_vel_global.vector.y = gaz_vel_global[4];
    msg_ang_vel_global.vector.z = gaz_vel_global[5];

    // Velocity Angular Local
    msg_ang_vel_local.vector.x = gaz_vel_local[3];
    msg_ang_vel_local.vector.y = gaz_vel_local[4];
    msg_ang_vel_local.vector.z = gaz_vel_local[5];

    // Position Angular
    msg_ang_pos.vector.x = gaz_pos[3];
    msg_ang_pos.vector.y = gaz_pos[4];
    msg_ang_pos.vector.z = gaz_pos[5];

    /* -------------------- */
    // Publish
    pub_state_truth_lin_ace.publish(msg_lin_ace);
    pub_state_truth_lin_vel_global.publish(msg_lin_vel_global);
    pub_state_truth_lin_vel_local.publish(msg_lin_vel_local);
    pub_state_truth_lin_pos.publish(msg_lin_pos);

    pub_state_truth_ang_ace.publish(msg_ang_ace);
    pub_state_truth_lin_ang_global.publish(msg_ang_vel_global);
    pub_state_truth_lin_ang_local.publish(msg_ang_vel_local);
    pub_state_truth_ang_pos.publish(msg_ang_pos);
}

/* ------------------------
    INTERACT WITH GAZEBO
------------------------ */

void ModelToGazebo::SetState()
{
    
    // Apply Force
    this->base_link->SetForce(math::Vector3d(force[0], force[1], force[2]));
    this->base_link->SetTorque(math::Vector3d(torque[0], torque[1], torque[2]));
}

void ModelToGazebo::GetState()
{

    // Position (x, y, z, roll, pitch, yaw) (Reference Frame GLOBAL)
    math::Pose3d pos_gaz = this->base_link->WorldCoGPose();

    // Velocity
    //Reference Frame LOCAL
    math::Vector3d lin_vel_gaz_local = this->base_link->RelativeLinearVel();
    math::Vector3d ang_vel_gaz_local = this->base_link->RelativeAngularVel();

    // Reference Frame GLOBAL
    math::Vector3d lin_vel_gaz_global = this->base_link->WorldLinearVel();
    math::Vector3d ang_vel_gaz_global = this->base_link->WorldAngularVel();

    // Acceleration
    // Reference Frame GLOBAL
    math::Vector3d lin_accel_gaz = this->base_link->WorldLinearAccel();
    math::Vector3d ang_accel_gaz = this->base_link->WorldAngularAccel();

    /* ----------------------------------
        Append Information 
    ---------------------------------- */
    // Position Linear
    gaz_pos[0] = pos_gaz.Pos()[0];
    gaz_pos[1] = pos_gaz.Pos()[1];
    gaz_pos[2] = pos_gaz.Pos()[2];

    // Position Angular
    gaz_pos[3] = pos_gaz.Rot().Roll();
    gaz_pos[4] = pos_gaz.Rot().Pitch();
    gaz_pos[5] = pos_gaz.Rot().Yaw();

    /* Global */
    // Velocity Linear
    gaz_vel_global[0] = lin_vel_gaz_global[0];
    gaz_vel_global[1] = lin_vel_gaz_global[1];
    gaz_vel_global[2] = lin_vel_gaz_global[2];

    // Velocity Angular
    gaz_vel_global[3] = ang_vel_gaz_global[0];
    gaz_vel_global[4] = ang_vel_gaz_global[1];
    gaz_vel_global[5] = ang_vel_gaz_global[2];

    /* Local */
    // Velocity Linear
    gaz_vel_local[0] = lin_vel_gaz_local[0];
    gaz_vel_local[1] = lin_vel_gaz_local[1];
    gaz_vel_local[2] = lin_vel_gaz_local[2];

    // Velocity Angular
    gaz_vel_local[3] = ang_vel_gaz_local[0];
    gaz_vel_local[4] = ang_vel_gaz_local[1];
    gaz_vel_local[5] = ang_vel_gaz_local[2];

    // NOTE: These values are taken directly from the force & torque applied by the model
    //       and the iteraction recorded by the API is added to correct collisions.
    // Reference Frame GLOBAL
    // Acceleration Linear
    gaz_ace[0] = force[0] / this->vehicle_model.params.mass + lin_accel_gaz[0];
    gaz_ace[1] = force[1] / this->vehicle_model.params.mass + lin_accel_gaz[1];
    gaz_ace[2] = force[2] / this->vehicle_model.params.mass + lin_accel_gaz[2];

    // Acceleration Angular
    gaz_ace[3] = torque[0] / this->vehicle_model.params.I(0,0) + ang_accel_gaz[0];
    gaz_ace[4] = torque[1] / this->vehicle_model.params.I(1,1) + ang_accel_gaz[1];
    gaz_ace[5] = torque[2] / this->vehicle_model.params.I(2,2) + ang_accel_gaz[2];
}

GZ_REGISTER_MODEL_PLUGIN(ModelToGazebo)
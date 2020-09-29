#include "gazebo_INS_devices.hh"

using namespace gazebo;

/////////////////////////////////////////////////
INS_devices::INS_devices() : ModelPlugin()
{
    accel.resize(6);
    vel.resize(6);
    pos.resize(6);
}

/////////////////////////////////////////////////
INS_devices::~INS_devices()
{
}

void INS_devices::Reset()
{
    ModelPlugin::Reset();
}

void INS_devices::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    if (!ros::isInitialized())
    {
        int argc = 0;
        char *argv = nullptr;
        ros::init(argc, &argv, _sdf->Get<std::string>("node_name"), ros::init_options::NoSigintHandler);
    }

    this->nh_.reset(new ros::NodeHandle(_sdf->Get<std::string>("node_name")));

    // Publisher State Truth
    this->pub_state_truth_accel = nh_->advertise<geometry_msgs::Vector3Stamped>("/sensors/gps/truth/acceleration", 1000);
    this->pub_state_truth_vel   = nh_->advertise<geometry_msgs::Vector3Stamped>("/sensors/gps/truth/vel", 1000);
    this->pub_state_truth_pos   = nh_->advertise<geometry_msgs::Vector3Stamped>("/sensors/gps/truth/pos", 1000);

    this->model_ = _parent;

    this->gznode = transport::NodePtr(new transport::Node());
    this->gznode->Init();
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&INS_devices::Update, this));

    dt_required_ = 1.0 / _sdf->Get<double>("rate");
}

void INS_devices::Update()
{

    std::lock_guard<std::mutex> lock(mutex);

    common::Time cur_time = model_->GetWorld()->SimTime();
    double dt;

    if (!isLoopTime(cur_time, dt))
    {
        return;
    }

    GetState();
    publish_state_truth();

    last_sim_time_ = cur_time;
}

void INS_devices::publish_state_truth()
{

    geometry_msgs::Vector3Stamped msg_accel = geometry_msgs::Vector3Stamped();
    geometry_msgs::Vector3Stamped msg_vel = geometry_msgs::Vector3Stamped();
    geometry_msgs::Vector3Stamped msg_pos = geometry_msgs::Vector3Stamped();

    // Header
    msg_accel.header.stamp = msg_vel.header.stamp = msg_pos.header.stamp = ros::Time::now();
    msg_accel.header.frame_id = msg_vel.header.frame_id = msg_pos.header.frame_id = "global";

    // Accelerometer
    msg_accel.vector.x = accel[0];
    msg_accel.vector.y = accel[1];
    msg_accel.vector.z = accel[2];

    // Velocity
    msg_vel.vector.x = vel[0];
    msg_vel.vector.y = vel[1];
    msg_vel.vector.z = vel[2];

    // Position
    msg_pos.vector.x = pos[0];
    msg_pos.vector.y = pos[1];
    msg_pos.vector.z = pos[2];

    pub_state_truth_accel.publish(msg_accel);
    pub_state_truth_vel.publish(msg_vel);
    pub_state_truth_pos.publish(msg_pos);
}

bool INS_devices::isLoopTime(const common::Time &time, double &dt)
{
    dt = (time - last_sim_time_).Double();
    if (dt < 0)
    {
        this->Reset();
        return false;
    }
    else if (dt >= dt_required_)
    {
        return true;
    }
    return false;
}

void INS_devices::GetState()
{

    // Position: x, y, z, roll, pitch, yaw
    ignition::math::Pose3d pos_gaz = model_->WorldPose();

    pos(0) = pos_gaz.Pos()[0];
    pos(1) = pos_gaz.Pos()[1];
    pos(2) = pos_gaz.Pos()[2];

    pos(3) = pos_gaz.Rot().Roll();
    pos(4) = pos_gaz.Rot().Pitch();
    pos(5) = pos_gaz.Rot().Yaw();

    // Velocity: vx, vy, vz + angular
    // Reference Frame Local (Vehicle)

    ignition::math::Vector3d lin_vel_gaz = model_->WorldLinearVel();
    ignition::math::Vector3d ang_vel_gaz = model_->WorldAngularVel();
    ignition::math::Vector3d lin_accel_gaz = model_->WorldLinearAccel();
    ignition::math::Vector3d ang_accel_gaz = model_->WorldAngularAccel();

    // Transformation Tools
    Eigen::Matrix3d matrix;

    Eigen::Vector3d lin_global_velocity, ang_global_velocity;
    Eigen::Vector3d lin_local_velocity, ang_local_velocity;
    Eigen::Vector3d lin_global_acceleration, ang_global_acceleration;
    Eigen::Vector3d lin_local_acceleration, ang_local_acceleration;


    // Unity Rotational Vector
    double l = 0; // x
    double m = 0; // y
    double n = 1; // z

    double inv_cos = 1 - cos(-pos(5)); // SIGN CHANGE
    double nor_cos = cos(-pos(5));     // SIGN CHANGE
    double nor_sin = sin(-pos(5));     // SIGN CHANGE

    // Rotation matrix to transform global to local velocities and accelerations. Angle of rotation --> -Yaw
    matrix << l * l * inv_cos + nor_cos, m * l * inv_cos - n * nor_sin, n * l * inv_cos + m * nor_sin,
              l * m * inv_cos + n * nor_sin, m * m * inv_cos + nor_cos, n * m * inv_cos - l * nor_sin,
              l * n * inv_cos - m * nor_sin, m * n * inv_cos + l * nor_sin, n * n * inv_cos + nor_cos;


    // Global => Local
    lin_global_velocity << lin_vel_gaz[0], lin_vel_gaz[1], lin_vel_gaz[2];
    lin_local_velocity = matrix * lin_global_velocity;

    ang_global_velocity << ang_vel_gaz[0], ang_vel_gaz[1], ang_vel_gaz[2];
    ang_local_velocity = matrix * ang_global_velocity;

    lin_global_acceleration << lin_accel_gaz[0], lin_accel_gaz[1], lin_accel_gaz[2];
    lin_local_acceleration = matrix * lin_global_acceleration;

    ang_global_acceleration << ang_accel_gaz[0], ang_accel_gaz[1], ang_accel_gaz[2];
    ang_local_acceleration = matrix * ang_global_acceleration;


    // Append Information
    accel(0) = lin_local_acceleration[0];
    accel(1) = lin_local_acceleration[1];
    accel(2) = lin_local_acceleration[2];

    accel(3) = ang_local_acceleration[0];
    accel(4) = ang_local_acceleration[1];
    accel(5) = ang_local_acceleration[2];

    vel(0) = lin_local_velocity[0];
    vel(1) = lin_local_velocity[1];
    vel(2) = lin_local_velocity[2];

    vel(3) = ang_local_velocity[0];
    vel(4) = ang_local_velocity[1];
    vel(5) = ang_local_velocity[2];
}

GZ_REGISTER_MODEL_PLUGIN(INS_devices)
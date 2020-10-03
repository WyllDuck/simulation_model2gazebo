#include <drone_model.h>

using namespace Eigen;

/* ------------------------
     INIT, DEL & RUN
------------------------ */

DronePhysicsModel::DronePhysicsModel(){
}

DronePhysicsModel::~DronePhysicsModel(){    
}

void DronePhysicsModel::run(VectorXd &_inputs, VectorXd &_ace1, VectorXd &_vel1, VectorXd &_pos1, VectorXd &ace2_, VectorXd &vel2_, VectorXd &pos2_)
{

    // Compute linear and angular accelerations.
    Vector3d a = acceleration(i, theta, xdot, m, g, k, kd);
    Vector3d omegadot = angular_acceleration(i, omega, I, L, b, k);

    omega = thetadot2omega(thetadot, theta);

    // Update State
    omega = omega + dt * omegadot;
    thetadot = omega2thetadot(omega, theta);
    theta = theta + dt * thetadot;
    xdot = xdot + dt * a;
    x = x + dt * xdot;
}

/* ------------------------
    AUXILIARY FUNCTIONS
------------------------ */
// NOTE: Inputs (_inputs) are values for ωi

// Compute thrust given current inputs and thrust coefficient.
void DronePhysicsModel::thrust(Vector3d &_inputs, Vector3d &T_)
{

    T_ << 0,
        0,
        k * sum(_inputs);
}

// Compute torques, given current inputs, length, drag coefficient, and thrust coefficient.
void DronePhysicsModel::torques(Vector3d &_inputs, Vector3d &tau_)
{

    tau_ << L * k * (_inputs[1] - _inputs[3]),
        L * k * (_inputs[2] - _inputw[4]),
        b * (_inputs[1] - _inputs[2] + _inputs[3] - _inputs[4])
}

// Compute acceleration in the global reference frame
void DronePhysicsModel::acceleration(Vector3d &_inputs, Vector3d &_theta, Vector3d &_xdot, Vector3d &a_)
{

    Vector3d T;
    Matrix3d R;

    R = rotation(angles, R);
    T = R * thrust(_inputs, T);
    Fd = -kd * xdot;

    a_ = gravity + 1 / m * T + Fd;
}

// Compute angular acceleration in the global reference frame
void DronePhysicsModel::angularAcceleration(Vector3d &_inputs, Vector3d &_omega, Vector3d &omegadot_)
{

    Vector3d tau;

    tau = torques(_inputs, tau);
    omegadot_ = inv(I) * (tau - cross(_omega, I * _omega));
}

// Compute the theta matrix
void DronePhysicsModel::thetadot2omega(Vector3d &_thetadot, Vector3d &_theta, Vector3d &omega_)
{

    // theta  <=> φ, θ, ψ
    //            0  1  2

    double c0, c1, s0, s1;

    c0 = cos(_theta[0]);
    c1 = cos(_theta[1]);

    s0 = sin(_theta[0]);
    s1 = sin(_theta[1]);

    Matrix3d thetadot2omegaMat;

    thetadot2omegaMat << 1, 0, −s1,
        0, c0, c1 * s0,
        0, −s0, c1 * c0;

    omega_ = thetadot2omegaMat * _thetadot;
}

// Compute the rotational matrix
void DronePhysicsModel::rotation(Vector3d &_theta, Matrix3d &R_)
{

    // theta  <=> φ, θ, ψ
    //            0  1  2

    double c0, c1, c2;
    double s0, s1, s2;

    c0 = cos(_theta[0]);
    c1 = cos(_theta[1]);
    c2 = cos(_theta[2]);

    s0 = sin(_theta[0]);
    s1 = sin(_theta[1]);
    s2 = sin(_theta[2]);

    R_ << c0 * c2 − c1 * s0 * s2,     −c2 * s0 − c0 * c1 * s2, s1 * s2,
        c1 * c2 * s0 + c0 * s2, c0 * c1 * c2 − s0 * s2,     −c2 * s1,
        s0 * s1, c0 * s1, c1;
}
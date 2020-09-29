#include <sim.h>

void Model::init (){

    // Initial simulation state.
    x       << 0, 0, 0;
    xdot    << 0, 0, 0;
    theta   << 0, 0, 0;

}

// Simulate some disturbance in the angular velocity.
// The magnitude of the deviation is in radians / second.
double deviation = 100;
thetadot = deg2rad(2 * deviation * rand(3, 1) - deviation);

// Step through the simulation, updating the state.
void Model::next_state(Eigen::VectorXd &_inputs){

    // Compute linear and angular accelerations.
    Eigen::Vector3d a = acceleration(i, theta, xdot, m, g, k, kd);
    Eigen::Vector3d omegadot = angular_acceleration(i, omega, I, L, b, k);

    omega = thetadot2omega(thetadot, theta);

    // Update State
    omega = omega + dt * omegadot;
    thetadot = omega2thetadot(omega, theta);
    theta = theta + dt * thetadot;
    xdot = xdot + dt * a;
    x = x + dt * xdot;

}

// NOTE: Inputs (_inputs) are values for ωi
// Compute thrust given current inputs and thrust coefficient.
void Model::thrust(Eigen::Vector3d &_inputs, Eigen::Vector3d &T_){
    
    T_ <<   0,
            0, 
            k * sum(_inputs);
    
}

// Compute torques, given current inputs, length, drag coefficient, and thrust coefficient.
void Model::torques(Eigen::Vector3d &_inputs, Eigen::Vector3d &tau_){
    
    tau_ << L * k * (_inputs[1] - _inputs[3]),
            L * k * (_inputs[2] - _inputw[4]),
            b * (_inputs[1] - _inputs[2] + _inputs[3] - _inputs[4])
    
}

// Compute acceleration in the global reference frame 
void Model::acceleration(Eigen::Vector3d &_inputs, Eigen::Vector3d &_theta, Eigen::Vector3d &_xdot, Eigen::Vector3d &a_){

    Eigen::Vector3d T;
    Eigen::Matrix3d R;

    R = rotation(angles, R);
    T = R * thrust(_inputs, T);
    Fd = -kd * xdot;

    a_ = gravity + 1 / m * T + Fd;

}

// Compute angular acceleration in the global reference frame 
void Model::angular_acceleration(Eigen::Vector3d &_inputs, Eigen::Vector3d &_omega, Eigen::Vector3d &omegadot_){

    Eigen::Vector3d tau;

    tau = torques(_inputs, tau);
    omegadot_ = inv(I) * (tau - cross(_omega, I * _omega));

}

// Compute the theta matrix
void Model::thetadot2omega(Eigen::Vector3d &_thetadot, Eigen::Vector3d &_theta, Eigen::Vector3d &omega_){

    // theta  <=> φ, θ, ψ
    //            0  1  2

    double c0, c1, s0, s1;

    c0 = cos(_theta[0]); 
    c1 = cos(_theta[1]);

    s0 = sin(_theta[0]); 
    s1 = sin(_theta[1]); 

    Eigen::Matrix3d thetadot2omegaMat;

    thetadot2omegaMat <<    1, 0, −s1,
                            0, c0, c1 * s0,
                            0, −s0, c1 * c0;

    omega_ = thetadot2omegaMat * _thetadot;

}

// Compute the rotational matrix
void Model::rotation(Eigen::Vector3d &_theta, Eigen::Matrix3d &R_){

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

    R_  <<  c0 * c2 − c1 * s0 * s2,     −c2 * s0 − c0 * c1 * s2,    s1 * s2, 
            c1 * c2 * s0 + c0 * s2,     c0 * c1 * c2 − s0 * s2,     −c2 * s1, 
            s0 * s1,                    c0 * s1,                    c1; 

}
// Purpose: Propagates relative motion of a chaser satellite w.r.t. a target in orbit using Hill/Clohessy-Wiltshire equations.

#include <iostream>
#include <Eigen/Dense>

// Structure to hold position and velocity vectors
struct State {
    Eigen::Vector3d pos;  // position [m]
    Eigen::Vector3d vel;  // velocity [m/s]
};

// One-step CW propagation (discrete-time)
State cwStep(State s, double n, double dt) {
    Eigen::Vector3d acc;

    // CW (Hill) equations in 3D (x, y, z)
    acc.x() = 3 * n * n * s.pos.x() + 2 * n * s.vel.y(); // radial acceleration
    acc.y() = -2 * n * s.vel.x();                        // along-track coupling
    acc.z() = -n * n * s.pos.z();                        // vertical motion

    // Update velocity using Euler integration
    s.vel += acc * dt;

    // Update position using Euler integration
    s.pos += s.vel * dt;

    return s;
}

int main() {
    double n = 0.0011; // mean motion [rad/s] (typical for LEO)
    double dt = 1.0;   // time step [s]

    // Initial conditions for chaser
    State s;
    s.pos = Eigen::Vector3d(10, 0, 0);      // [m]
    s.vel = Eigen::Vector3d(0, 0.05, 0);    // [m/s]

    // Compute next state after 1 second
    State s_next = cwStep(s, n, dt);

    // Output new position and velocity
    std::cout << "Next position: " << s_next.pos.transpose() << std::endl;
    std::cout << "Next velocity: " << s_next.vel.transpose() << std::endl;

    return 0;
}

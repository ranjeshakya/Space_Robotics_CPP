#include <iostream>
#include <Eigen/Geometry>

int main() {
    // Desired orientation: no rotation (identity quaternion)
    Eigen::Quaterniond q_d(1, 0, 0, 0); // [w, x, y, z]

    // Current orientation: 30° rotation about X-axis
    Eigen::Quaterniond q_c(0.9659, 0.2588, 0, 0); // ~30 deg about X

    // Compute quaternion error: q_err = q_d * q_c⁻¹
    Eigen::Quaterniond q_err = q_d * q_c.inverse();

    // Convert quaternion error into axis-angle representation
    Eigen::AngleAxisd rot_vec(q_err.normalized());

    // Proportional gain for control
    double Kp = 2.0;

    // Control torque = Kp * angle * axis
    Eigen::Vector3d torque = Kp * rot_vec.angle() * rot_vec.axis();

    // Output the corrective torque vector
    std::cout << "Control torque: " << torque.transpose() << std::endl;

    return 0;
}

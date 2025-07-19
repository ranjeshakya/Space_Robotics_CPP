#include<iostream>

#include<Eigen/Dense>



int main(){
    // Joint velocity vector: 2 DOF manipulator

    Eigen::Vector2d q_dot(0.2,-0.1);  //[Rad/sec]

    // Jacobian matrix (3x2) mapping joint valocity to end effector velocity

    Eigen::MatrixXd J(3,2);
    J << 0.5, 0.3,
         0.2, 0.4,
         0.0, 0.1;

// Inverse of the base inertia matrix (3x3) [1/kg-m^2]

Eigen::Matrix3d M_base_inv;
M_base_inv << 10, 0, 0, 
              0 , 15, 0,
              0, 0, 20;

// Calculate base (spacecraft) reaction velocity due to manipulator motion
// base_velocity = -M_base^-1 *J*q_dot

Eigen::Vector3d base_vel = -M_base_inv *J*q_dot;

// output the result
std::cout << "Base rection vecloity: \n" << base_vel << std::endl;

return 0;
}
#include "grav_comp.h"

#include <iostream>

void printFrame(const Eigen::Isometry3d& fr) {
    std::cout << fr(0,0) << ", " << fr(0,1) << ", " << fr(0,2) << ", " << fr(0,3) << std::endl;
    std::cout << fr(1,0) << ", " << fr(1,1) << ", " << fr(1,2) << ", " << fr(1,3) << std::endl;
    std::cout << fr(2,0) << ", " << fr(2,1) << ", " << fr(2,2) << ", " << fr(2,3) << std::endl;
    std::cout << fr(3,0) << ", " << fr(3,1) << ", " << fr(3,2) << ", " << fr(3,3) << std::endl;
}

// q is a 8x1 vector of joint positions (q_col, q_1, q_2, ...)
// result is the 8x1 vector of gravity compensation forces (tau_g_col, tau_g_1, tau_g_2, ...)
void computeGravComp(const Eigen::VectorXd& q, Eigen::VectorXd& result) {

    const Eigen::Vector3d grav(0,0,-9.8);

    const std::array<Eigen::Vector3d, 7> arr_i_Ci_P({
        Eigen::Vector3d(0, -0.06, 0.13),
        Eigen::Vector3d(0, 0.06, 0.07),
        Eigen::Vector3d(0, 0.06, 0.13),
        Eigen::Vector3d(0, -0.06, 0.07),
        Eigen::Vector3d(0, 0, 0.124),
        Eigen::Vector3d(0, 0, 0.0625),
        Eigen::Vector3d(0.156521377991514, 0.0108632966681013, -0.0694477283378301)});

    const std::array<double, 7> arr_Mi({2, 2, 2, 2, 2, 0.2, 1.53331924});

    const std::array<Eigen::Vector3d, 7> arr_i_i_z({
        Eigen::Vector3d(0, 0, 1),
        Eigen::Vector3d(0, 1, 0),
        Eigen::Vector3d(0, 0, 1),
        Eigen::Vector3d(0, -1, 0),
        Eigen::Vector3d(0, 0, 1),
        Eigen::Vector3d(0, 1, 0),
        Eigen::Vector3d(0, 0, 1)});

    Eigen::Matrix4d T_0_1, T_1_2, T_2_3, T_3_4, T_4_5, T_5_6, T_6_7, T_7_8;
    T_0_1 << cos(q[0]), -sin(q[0]), 0, 0,
            sin(q[0]), cos(q[0]), 0, 0,
            0, 0, 1, 0.1,
            0, 0, 0, 1;

    T_1_2 << -sin(q[1]), -cos(q[1]), 0, 0,
            0.5*cos(q[1]), -0.5*sin(q[1]), -0.866025404, -0.0955,
            0.866025404*cos(q[1]), -0.866025404*sin(q[1]), 0.5, 1.2283,
            0, 0, 0, 1;

    T_2_3 << cos(q[2]), 0, sin(q[2]), 0.0,
            0, 1, 0, 0,
            -sin(q[2]), 0, cos(q[2]), 0.2005,
            0, 0, 0, 1;

    T_3_4 << cos(q[3]), -sin(q[3]), 0, 0,
            sin(q[3]), cos(q[3]), 0, 0,
            0, 0, 1, 0.2,
            0, 0, 0, 1;

    T_4_5 << cos(q[4]), 0, -sin(q[4]), 0.0,
            0, 1, 0, 0,
            sin(q[4]), 0, cos(q[4]), 0.2,
            0, 0, 0, 1;

    T_5_6 << cos(q[5]), -sin(q[5]), 0, 0,
            sin(q[5]), cos(q[5]), 0, 0,
            0, 0, 1, 0.2,
            0, 0, 0, 1;

    T_6_7 << cos(q[6]), 0, sin(q[6]), 0.0,
            0, 1, 0, 0,
            -sin(q[6]), 0, cos(q[6]), 0.19,
            0, 0, 0, 1;

    T_7_8 << cos(q[7]), -sin(q[7]), 0, 0,
            sin(q[7]), cos(q[7]), 0, 0,
            0, 0, 1, 0.078,
            0, 0, 0, 1;

    std::array<Eigen::Matrix4d, 8> arr_T_i_i({T_0_1, T_1_2, T_2_3, T_3_4, T_4_5, T_5_6, T_6_7, T_7_8});

    std::array<Eigen::Isometry3d, 7> arr_fk;
    arr_fk[0] = T_0_1 * T_1_2;
    //std::cout << "FK for i=" << 0 << ":" << std::endl;
    //printFrame(arr_fk[0]);
    // Calculate forward kinematics
    for (int i = 1; i < 7; ++i) {
        arr_fk[i] = arr_fk[i-1] * arr_T_i_i[i+1];
        //std::cout << "FK for i=" << i << ":" << std::endl;
        //printFrame(arr_fk[i]);
    }

    // Calculate gravity compensation forces
    Eigen::Vector3d i1_i1_F(0,0,0);
    Eigen::Vector3d i1_i1_N(0,0,0);
    for (int i = 6; i >= 0; --i) {
        // Position of center of mass of link i
        const Eigen::Vector3d& i_Ci_P = arr_i_Ci_P[i];

        // Mass of link i
        const double& Mi = arr_Mi[i];

        // i_i_z - joint axis
        const Eigen::Vector3d& i_i_z = arr_i_i_z[i];

        Eigen::Isometry3d i_i1_T;

        const Eigen::Isometry3d& B_i_T(arr_fk[i]);

        Eigen::Vector3d i_g = B_i_T.inverse().rotation() * grav;

        if (i == 6) {
            i_i1_T.setIdentity();
        }
        else {
            i_i1_T = arr_T_i_i[i+2];
        }

        Eigen::Isometry3d::LinearMatrixType i_i1_R = i_i1_T.rotation();
        Eigen::Vector3d i_i1_P = i_i1_T.translation();

        Eigen::Vector3d i_i_F = i_i1_R * i1_i1_F - Mi * i_g;

        // i_i_N - inter-link interaction torques
        Eigen::Vector3d i_i_N = i_i1_R * i1_i1_N + i_Ci_P.cross(i_i_F) + (i_i1_P - i_Ci_P).cross(i_i1_R * i1_i1_F);

        double torque_i = i_i_N[0]*i_i_z[0] + i_i_N[1]*i_i_z[1] + i_i_N[2]*i_i_z[2];
        result[i] = torque_i;

        i1_i1_F = i_i_F;
        i1_i1_N = i_i_N;
    }
}

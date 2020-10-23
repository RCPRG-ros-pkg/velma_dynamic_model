// Copyright (c) 2020, Robot Control and Pattern Recognition Group,
// Institute of Control and Computation Engineering
// Warsaw University of Technology
//
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Warsaw University of Technology nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: Dawid Seredynski
//

#include <ros/ros.h>
#include "ros/package.h"
#include <tf/transform_broadcaster.h>

#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include <velma_dynamic_model/velma_dynamic_model.h>

void publishTransform(tf::TransformBroadcaster &br, const Eigen::Isometry3d &T,
                                 const std::string &frame_id, const std::string &base_frame_id) {
    Eigen::Quaterniond q(T.rotation());
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(T.translation().x(), T.translation().y(), T.translation().z()));
    transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_frame_id, frame_id));
}

double getInterval(struct timespec &t1, struct timespec &t2) {
    int sec = t2.tv_sec - t1.tv_sec;
    int nsec = int(t2.tv_nsec) - int(t1.tv_nsec);
    if (nsec < 0) {
        nsec += 1000000000;
        sec += 1;
    }
    return double(sec) + double(nsec)/1000000000.0;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "velma_dynamic_model_test");
    ros::NodeHandle n;

    // Wait until robot_description ROS param is available
    while (ros::ok()) {
        std::string robot_description;
        if (n.getParam("/robot_description", robot_description)) {
            break;
        }
        std::cout << "Waiting for \"robot_description\" ROS parameter." << std::endl;
        ros::Duration(1.0).sleep();
    }

    VelmaDynamicModelPtr model = VelmaDynamicModel::createFromRosParam();
    if (!model) {
        std::cout << "ERROR: could not create model." << std::endl;
        return -1;
    }
    tf::TransformBroadcaster br;

    std::vector<std::string > link_names = model->getLinkNames();
    std::cout << "Links:" << std::endl;
    for (int i = 0; i < link_names.size(); ++i) {
        std::cout << "    " << link_names[i] << std::endl;
    }

    std::vector<std::string > moveable_joints = {
        "torso_0_joint",
        "right_arm_0_joint",
        "right_arm_1_joint",
        "right_arm_2_joint",
        "right_arm_3_joint",
        "right_arm_4_joint",
        "right_arm_5_joint",
        "right_arm_6_joint",
        "left_arm_0_joint",
        "left_arm_1_joint",
        "left_arm_2_joint",
        "left_arm_3_joint",
        "left_arm_4_joint",
        "left_arm_5_joint",
        "left_arm_6_joint",
    };
    std::vector<double > joint_pos_des = {0, -0.3, -1.8, 1.25, 0.85, 0, -0.5, 0,
                                                        0.3, 1.8, -1.25, -0.85, 0, 0.5, 0};
    std::vector<int> moveable_joints_idx_map;
    for (int mov_jnt_idx = 0; mov_jnt_idx < moveable_joints.size(); ++mov_jnt_idx) {
        const std::string& moveable_joint_name = moveable_joints[mov_jnt_idx];
        dart::dynamics::Joint *joint = model->getSkeleton()->getJoint(moveable_joint_name);
        int q_idx = joint->getIndexInSkeleton(0);
        moveable_joints_idx_map.push_back( q_idx );
        std::cout << "Joint \"" << moveable_joint_name << "\" has index " << q_idx << std::endl;
    }

    double stiffness = 2.0;
    double damping = 2.0;

    ros::Rate loop_rate(500);
    clockid_t clock_id = CLOCK_REALTIME;
    //clockid_t clock_id = CLOCK_MONOTONIC;
    //clockid_t clock_id = CLOCK_PROCESS_CPUTIME_ID;
    struct timespec prev_t1;
    while (ros::ok()) {
        Eigen::VectorXd grav_forces = model->getSkeleton()->getGravityForces();
        Eigen::VectorXd cmd_ext_forces(grav_forces.size());
        cmd_ext_forces.setZero();
        Eigen::VectorXd pos = model->getSkeleton()->getPositions();
        Eigen::VectorXd vel = model->getSkeleton()->getVelocities();

        for (int mov_jnt_idx = 0; mov_jnt_idx < moveable_joints.size(); ++mov_jnt_idx) {
            int q_idx = moveable_joints_idx_map[mov_jnt_idx];
            double err = joint_pos_des[mov_jnt_idx] - pos[q_idx];
            double force = stiffness*err - vel[q_idx]*damping;
            cmd_ext_forces[q_idx] = force;
        }

        model->getSkeleton()->setForces( grav_forces + cmd_ext_forces );
        struct timespec t1, t2;
        clock_gettime(clock_id, &t1);

        model->step();
        //grav_forces = model->getSkeleton()->getGravityForces();
        //Eigen::VectorXd coriolis_forces = model->getSkeleton()->getCoriolisForces();

        clock_gettime(clock_id, &t2);
        double step_time_ms = 1000.0*getInterval(t1, t2);
        //std::cout << step_time_ms << std::endl;
        //std::cout << getInterval(prev_t1, t1) << std::endl;
        prev_t1 = t1;

        for (int i = 0; i < link_names.size(); ++i) {
            Eigen::Isometry3d T_B_L;
            model->getFk(link_names[i], T_B_L);
            publishTransform(br, T_B_L, link_names[i], "world");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}



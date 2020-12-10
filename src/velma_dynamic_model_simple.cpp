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

#include <string>
#include <stdlib.h>
#include <stdio.h>

#include "Eigen/Dense"
#include <dart/utils/PackageResourceRetriever.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>

#include <velma_dynamic_model/velma_dynamic_model_simple.h>

std::vector<dart::dynamics::BodyNode* > getAllChildNodes(dart::dynamics::BodyNode* bn) {
    std::vector<dart::dynamics::BodyNode* > result;
    for (int ch_idx = 0; ; ++ch_idx) {
        dart::dynamics::BodyNode* ch_bn = bn->getChildBodyNode(ch_idx);
        if (ch_bn == NULL) {
            break;
        }
        result.push_back(ch_bn);
        std::vector<dart::dynamics::BodyNode* > ch_result = getAllChildNodes(ch_bn);
        result.insert( result.end(), ch_result.begin(), ch_result.end() );
    }
    return result;
}

void calculateCOM(const dart::dynamics::BodyNode* base_bn,
                                    const std::vector<dart::dynamics::BodyNode*>& bn_list,
                                    Eigen::Vector3d& out_COM, double& out_MASS) {
    Eigen::Isometry3d base_tf = base_bn->getTransform();
    out_COM.setZero();
    out_MASS = 0.0;
    for (int i = 0; i < bn_list.size(); ++i) {
        if (!bn_list[i]->getGravityMode()) {
            std::cout << "gravity is disabled for " << bn_list[i]->getName() << std::endl;
            continue;
        }
        const Eigen::Vector3d& com = bn_list[i]->getLocalCOM();
        Eigen::Isometry3d tf = bn_list[i]->getTransform();
        out_COM = out_COM + tf * com * bn_list[i]->getMass();
        out_MASS += bn_list[i]->getMass();
    }
    out_COM = out_COM * (1.0 / out_MASS);
    out_COM = base_tf.inverse() * out_COM;
}

VelmaDynamicModelSimplePtr VelmaDynamicModelSimple::createFromRosParam(
                                const std::vector<std::string >& controlled_joints,
                                const Eigen::VectorXd& damping, const Eigen::VectorXd& inertia) {
    // Add paths of ROS packages
    dart::utils::PackageResourceRetrieverPtr resource_retriever(
                                                    new dart::utils::PackageResourceRetriever());
    const std::vector<std::string > required_packages = {
            "velma_description",
            "lwr_defs",
            "barrett_hand_defs",
        };
    for (int i = 0; i < required_packages.size(); ++i) {
        std::string path = ros::package::getPath(required_packages[i]);
        if (path.empty()) {
            std::cout << "ERROR: Path for package \"" << required_packages[i]
                        << "\" could not be found."<< std::endl;
            return VelmaDynamicModelSimplePtr();
        }
        //std::cout << "Package \"" << required_packages[i] << "\" is in path " << path <<std::endl;
        resource_retriever->addPackageDirectory(required_packages[i], path);
    }

    // Get robot description from ROS parameter server
    ros::NodeHandle nh;
    std::string robot_description;
    if (!nh.getParam("/robot_description", robot_description)) {
        std::cout << "ERROR: Could not get \"robot_description\" ROS parameter" << std::endl;
        return VelmaDynamicModelSimplePtr();
    }

    dart::utils::DartLoader loader;
    dart::dynamics::SkeletonPtr sk = loader.parseSkeletonString(robot_description,
                                                        dart::common::Uri(), resource_retriever);

    VelmaDynamicModelSimplePtr result( new VelmaDynamicModelSimple(sk, controlled_joints,
                                                                            damping, inertia) );

    return result;
}

std::ostream &operator<<(std::ostream &os, Eigen::Vector3d const &m) {
    return os << "(" << m[0] << ", " << m[1] << ", " << m[2] << ")";
}

std::ostream &operator<<(std::ostream &os, Eigen::Quaterniond const &m) {
    return os << "(" << m.x() << ", " << m.y() << ", " << m.z() << ", " << m.w() << ")";
}

std::ostream &operator<<(std::ostream &os, Eigen::Isometry3d const &m) {
    Eigen::Quaterniond q(m.rotation());
    Eigen::Vector3d p = m.translation();
    return os << "(" << p << ", " << q << ")";
}

// This method is used to verify equations in the report and to compare it with DART
void VelmaDynamicModelSimple::computeGravComp(Eigen::VectorXd& result) {
    Eigen::Vector3d i1_i1_F(0,0,0);
    Eigen::Vector3d i1_i1_N(0,0,0);
    Eigen::Isometry3d B_i1_T;

    for (int i = 6; i >= 0; --i) {
        dart::dynamics::BodyNode* bn = bn_list_[i];
        Eigen::Vector3d i_Ci_P;
        double Mi;
        Eigen::Isometry3d i_i1_T;

        Eigen::Isometry3d B_i_T = bn->getTransform();
        Eigen::Vector3d i_g = B_i_T.inverse().rotation() * getGravVector();

        if (i == 6) {
            i_Ci_P = com_ee_;
            Mi = mass_ee_;
            i_i1_T.setIdentity();
        }
        else {
            i_Ci_P = bn->getLocalCOM();
            Mi = bn->getMass();
            i_i1_T = B_i_T.inverse() * B_i1_T;
        }

        Eigen::Isometry3d::LinearMatrixType i_i1_R = i_i1_T.rotation();
        Eigen::Vector3d i_i1_P = i_i1_T.translation();

        dart::dynamics::RevoluteJoint* joint = static_cast<dart::dynamics::RevoluteJoint*>( bn->getParentJoint() );
        Eigen::Vector3d i_i_z = joint->getAxis();


        Eigen::Vector3d i_i_F = i_i1_R * i1_i1_F - Mi * i_g;

        //std::cout << "idx: " << i << ", i_Ci_P=" << i_Ci_P << ", i_i_F=" << i_i_F
        //        << ", i_Ci_P x i_i_F=" << (i_Ci_P.cross(i_i_F)) << std::endl;


        Eigen::Vector3d i_i_N = i_i1_R * i1_i1_N + i_Ci_P.cross(i_i_F) + (i_i1_P - i_Ci_P).cross(i_i1_R * i1_i1_F);

        //std::cout << "i: " << i << ", Mi: " << Mi
        //        << ", i_Ci_P: " << i_Ci_P[0] << ", " << i_Ci_P[1] << ", " << i_Ci_P[2]
        //        << ", i_i1_P: " << i_i1_P[0] << ", " << i_i1_P[1] << ", " << i_i1_P[2]
        //        << ", i_i_z: " << i_i_z[0] << ", " << i_i_z[1] << ", " << i_i_z[2] << std::endl;

        // i_i_N - inter-link interaction torques
        // i_i_z - joint axis
        double torque_i = i_i_N[0]*i_i_z[0] + i_i_N[1]*i_i_z[1] + i_i_N[2]*i_i_z[2];
        result[i] = torque_i;

        i1_i1_F = i_i_F;
        i1_i1_N = i_i_N;
        B_i1_T = B_i_T;
    }
}

VelmaDynamicModelSimple::VelmaDynamicModelSimple(dart::dynamics::SkeletonPtr &sk,
        const std::vector<std::string >& controlled_joints,
        const Eigen::VectorXd& damping, const Eigen::VectorXd& inertia)
: VelmaDynamicModelBase(sk, controlled_joints)
, damping_(damping)
, inertia_(inertia)
, pos_(controlled_joints.size())
, vel_(controlled_joints.size())
, forces_(controlled_joints.size())
, grav_forces_(controlled_joints.size()) {
    pos_.setZero();
    vel_.setZero();
    forces_.setZero();
    grav_forces_.setZero();


    dart::dynamics::BodyNode* a7 = sk->getBodyNode("right_arm_7_link");
    std::cout << "Child nodes of " << a7->getName() << std::endl;
    std::vector<dart::dynamics::BodyNode* > ee_nodes = getAllChildNodes(a7);
    ee_nodes.push_back( a7 );
    for (int i = 0; i < ee_nodes.size(); ++i) {
        dart::dynamics::BodyNode* bn = ee_nodes[i];
        const Eigen::Vector3d &com = bn->getLocalCOM();
        std::cout << "  " << bn->getName() << ": com: (" << com[0] << ","
                << com[1] << "," << com[2] << "), mass: " << bn->getMass()
                << ", grav: " << bn->getGravityMode() << std::endl;
    }
    calculateCOM(a7, ee_nodes, com_ee_, mass_ee_);
    std::cout << "  total: com: (" << com_ee_[0] << ","
            << com_ee_[1] << "," << com_ee_[2] << "), mass: " << mass_ee_ << std::endl;

    std::vector<std::string > link_names({"right_arm_1_link", "right_arm_2_link",
            "right_arm_3_link", "right_arm_4_link", "right_arm_5_link",
            "right_arm_6_link", "right_arm_7_link"});

    for (int i = 0; i < link_names.size(); ++i) {
        bn_list_.push_back( sk->getBodyNode(link_names[i]) );
    }

}

void VelmaDynamicModelSimple::getGravityForces(Eigen::VectorXd &result) const {
    for (int jnt_idx = 0; jnt_idx < controlled_joints_idx_map_.size(); ++jnt_idx) {
        int q_idx = controlled_joints_idx_map_[jnt_idx];
        sk_->setPosition(q_idx, pos_[jnt_idx]);
    }
    const Eigen::VectorXd& grav_forces = sk_->getGravityForces();
    for (int jnt_idx = 0; jnt_idx < controlled_joints_idx_map_.size(); ++jnt_idx) {
        int q_idx = controlled_joints_idx_map_[jnt_idx];
        result[jnt_idx] = grav_forces[q_idx];
    }    
}

void VelmaDynamicModelSimple::getCoriolisForces(Eigen::VectorXd &result) const {
    for (int jnt_idx = 0; jnt_idx < controlled_joints_idx_map_.size(); ++jnt_idx) {
        result[jnt_idx] = 0.0;
    }    
}

void VelmaDynamicModelSimple::getPositions(Eigen::VectorXd &result) const {
    for (int jnt_idx = 0; jnt_idx < controlled_joints_idx_map_.size(); ++jnt_idx) {
        result[jnt_idx] = pos_[jnt_idx];
    }
}

void VelmaDynamicModelSimple::setPositions(const Eigen::VectorXd &pos) {
    for (int jnt_idx = 0; jnt_idx < controlled_joints_idx_map_.size(); ++jnt_idx) {
        pos_[jnt_idx] = pos[jnt_idx];
    }
}

void VelmaDynamicModelSimple::getVelocities(Eigen::VectorXd &result) const {
    for (int jnt_idx = 0; jnt_idx < controlled_joints_idx_map_.size(); ++jnt_idx) {
        result[jnt_idx] = vel_[jnt_idx];
    }
}

void VelmaDynamicModelSimple::setVelocities(const Eigen::VectorXd &vel) {
    for (int jnt_idx = 0; jnt_idx < controlled_joints_idx_map_.size(); ++jnt_idx) {
        vel_[jnt_idx] = vel[jnt_idx];
    }
}

void VelmaDynamicModelSimple::setForces(const Eigen::VectorXd &force) {
    for (int jnt_idx = 0; jnt_idx < controlled_joints_idx_map_.size(); ++jnt_idx) {
        forces_[jnt_idx] = force[jnt_idx];
    }
}

void VelmaDynamicModelSimple::step() {
    getGravityForces( grav_forces_ );
    for (int jnt_idx = 0; jnt_idx < controlled_joints_idx_map_.size(); ++jnt_idx) {
        double force = forces_[jnt_idx] - grav_forces_[jnt_idx] - vel_[jnt_idx]*damping_[jnt_idx];
        vel_[jnt_idx] = vel_[jnt_idx] + force / inertia_[jnt_idx] * time_step_;
        pos_[jnt_idx] = pos_[jnt_idx] + vel_[jnt_idx] * time_step_;
    }
}


/*
Mi = bn->getMass()
const Eigen::Vector3d &  = bn->getLocalCOM();  // Return center of mass expressed in body frame. 

i_i_F = i_i1_R * i1_i1_F - Mi * i_g;

i_i_N = i_i1_R * i1_i1_N + i_Ci_P.cross(i_i_F) + (i_i1_P - i_Ci_P).cross(i_i1_R * i1_i1_F);
// i_i_N - inter-link interaction torques
// i_i_z - joint axis
double torque_i = KDL::dot(i_i_N, i_i_z);
*/
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

#include <velma_dynamic_model/velma_dynamic_model_simple.h>

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
    const Eigen::VectorXd& grav_forces = sk_->getGravityForces();

    getGravityForces( grav_forces_ );

    for (int jnt_idx = 0; jnt_idx < controlled_joints_idx_map_.size(); ++jnt_idx) {
        double force = forces_[jnt_idx] - grav_forces_[jnt_idx] - vel_[jnt_idx]*damping_[jnt_idx];
        vel_[jnt_idx] = vel_[jnt_idx] + force / inertia_[jnt_idx] * time_step_;
        pos_[jnt_idx] = pos_[jnt_idx] + vel_[jnt_idx] * time_step_;
    }
}

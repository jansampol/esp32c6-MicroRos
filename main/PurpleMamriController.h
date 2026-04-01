#pragma once

#include "KinematicsController.h"

class PurpleMamriController : public KinematicsController {
public:
    PurpleMamriController() { 
        // Set up H matrices and twists
        setupGeometry(); 

        dof_ = twists_.size();

        // Precompute tilde twists
        tildeTwists_.clear();
        tildeTwists_.reserve(dof_);
        for (const auto& t : twists_) tildeTwists_.push_back(twistToTilde(t));
    }

    // Explicitly override the destructor
    ~PurpleMamriController() override = default;

    // Override the IK with the specific analytic solution for this robot
    std::vector<std::vector<int>> inverseKinematics(const Matrix4f& H_desired) const override;

    void setEndEffector(EndEffector ee) override;

    Matrix4f targetToHomogeneous(const std::vector<float> &target) const override;
private:
    void setupGeometry(); // Helper to setup the H matrices and twists
};
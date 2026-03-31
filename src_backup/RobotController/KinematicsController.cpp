#include "KinematicsController.h"

#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <Eigen/Geometry>



// === Forward kinematics: end-effector =======================================

KinematicsController::Matrix4f
KinematicsController::forwardKinematicsEE(const std::vector<int> &jointSteps) const
{

    std::vector<float> jointAnglesRad = stepsToRad(jointSteps);

    Matrix4f T = Matrix4f::Identity();

    for (std::size_t i = 0; i < dof_; ++i)
    {
        T = T * matrixExponential(tildeTwists_[i], jointAnglesRad[i]);
    }

    return T * H_ee0_;
}

// === Forward kinematics: all joint frames ===================================

std::vector<KinematicsController::Matrix4f>
KinematicsController::forwardKinematicsAll(const std::vector<int> &jointSteps) const
{
    std::vector<float> jointAnglesRad = stepsToRad(jointSteps);

    std::vector<Matrix4f> frames;
    frames.reserve(dof_ + 1);

    Matrix4f T = Matrix4f::Identity();

    // Joint frames
    for (std::size_t i = 0; i < dof_; ++i)
    {
        T = T * matrixExponential(tildeTwists_[i], jointAnglesRad[i]);
        frames.push_back(T * H_joints0_[i]); // joint i frame in space
    }
    // End-effector frame at the end
    frames.push_back(T * H_ee0_);

    return frames;
}

KinematicsController::Matrix4f KinematicsController::targetToHomogeneous(const std::vector<float> &target) const {
    KinematicsController::Matrix4f H = KinematicsController::Matrix4f::Identity();
    if (target.size() < 6) return H;

    // target: [x, y, z, pitch, yaw, roll]
    float x = target[0];
    float y = target[1];
    float z = target[2];
    float pitch = target[3];
    float yaw = target[4];
    float roll = target[5];

    Eigen::AngleAxisf Rz(yaw, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf Ry(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf Rx(roll, Eigen::Vector3f::UnitX());

    Eigen::Matrix3f R = (Rz * Ry * Rx).toRotationMatrix();
    H.block<3,3>(0,0) = R;
    H.block<3,1>(0,3) = Eigen::Vector3f(x, y, z);
    return H;
}


std::vector<float> KinematicsController::homogenousToPose(const Matrix4f &H) const {
    // Result vector
    std::vector<float> pose(6);

    // Extract translation
    pose[0] = H(0, 3);
    pose[1] = H(1, 3);
    pose[2] = H(2, 3);

    // Extract rotation matrix
    Matrix3f R = H.block<3, 3>(0, 0);

    // Get the Euler angles
    Vector3f angles = R.eulerAngles(2, 1, 0);

    // Extract the Euler angles
    pose[3] = angles[1]; // pitch
    pose[4] = angles[0]; // yaw
    pose[5] = angles[2]; // roll

    return pose;
}


std::vector<int> KinematicsController::pickClosestSolution(
    const std::vector<std::vector<int>> &possible,
    const std::vector<int> &current) const
{
    // No IK solutions → stay where we are
    if (possible.empty())
        return current;

    int bestCost = std::numeric_limits<int>::max();
    std::vector<int> best = current;

    for (const auto &sol : possible)
    {
        int cost = 0;

        for (size_t i = 0; i < sol.size(); ++i)
            cost += std::abs(sol[i] - current[i]);

        if (cost < bestCost)
        {
            bestCost = cost;
            best = sol;
        } else if (cost == bestCost) {
            // prefer J3 positive
            if (sol[2] > best[2]) {
                best = sol;
            }
        }
    }

    return best;
}


// === Product of exponentials helpers ========================================

// Computes the tilde form of a twist: [S] = [[w]  v; 0  0]
KinematicsController::Matrix4f
KinematicsController::twistToTilde(const Vector6f &twist) const
{
    // twist = [w_x, w_y, w_z, v_x, v_y, v_z]^T
    Vector3f w(twist(0), twist(1), twist(2));
    Vector3f v(twist(3), twist(4), twist(5));

    Matrix3f wSkew;
    wSkew << 0.0f, -w(2), w(1),
        w(2), 0.0f, -w(0),
        -w(1), w(0), 0.0f;

    Matrix4f Ttilde = Matrix4f::Zero();
    Ttilde.block<3, 3>(0, 0) = wSkew;
    Ttilde.block<3, 1>(0, 3) = v;
    // last row stays zeros for a twist
    return Ttilde;
}

// Computes exp([S] * theta) using standard twist exponential formula
KinematicsController::Matrix4f
KinematicsController::matrixExponential(const Matrix4f &tildeTwist, float theta) const
{
    Matrix4f result = Matrix4f::Identity();

    Matrix3f wSkew = tildeTwist.block<3, 3>(0, 0);
    Vector3f v = tildeTwist.block<3, 1>(0, 3);

    // Extract w from its skew-symmetric matrix
    Vector3f w;
    w << wSkew(2, 1), wSkew(0, 2), wSkew(1, 0);

    const float wNorm = w.norm();

    // Prismatic joint: pure translation
    if (wNorm < 1e-6f)
    {
        result.block<3, 3>(0, 0) = Matrix3f::Identity();
        result.block<3, 1>(0, 3) = v * theta;
        return result;
    }

    // Revolute joint
    Vector3f wUnit = w / wNorm;

    Matrix3f wUnitSkew;
    wUnitSkew << 0.0f, -wUnit(2), wUnit(1),
        wUnit(2), 0.0f, -wUnit(0),
        -wUnit(1), wUnit(0), 0.0f;

    const float th = wNorm * theta; // if w is unit, th ≈ theta

    Matrix3f I = Matrix3f::Identity();

    // Rodrigues formula for rotation
    Matrix3f R = I + std::sin(th) * wUnitSkew + (1.0f - std::cos(th)) * (wUnitSkew * wUnitSkew);

    // V(theta) for translation: p = V(theta) * v
    Matrix3f V = I * th + (1.0f - std::cos(th)) * wUnitSkew + (th - std::sin(th)) * (wUnitSkew * wUnitSkew);

    Vector3f p = V * v;

    result.block<3, 3>(0, 0) = R;
    result.block<3, 1>(0, 3) = p;

    return result;
}

// === steps ↔ radians conversion helpers =====================================

std::vector<float>
KinematicsController::stepsToRad(const std::vector<int> &steps) const
{

    std::vector<float> rad(dof_);
    const float twoPi = 2.0f * PI_FLOAT;

    for (std::size_t i = 0; i < dof_; ++i)
    {
        rad[i] = steps[i] * (twoPi / stepsPerRev_[i]);
    }
    return rad;
}

std::vector<int>
KinematicsController::radToSteps(const std::vector<float> &jointAngles) const
{

    std::vector<int> steps(dof_);
    const float twoPi = 2.0f * PI_FLOAT;

    for (std::size_t i = 0; i < dof_; ++i)
    {
        float factor = twoPi / stepsPerRev_[i]; // rad per step
        steps[i] = static_cast<int>(std::round(jointAngles[i] / factor));
    }
    return steps;
}

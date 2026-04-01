#pragma once

#ifdef B0
#undef B0
#endif
#ifdef B1
#undef B1
#endif
#ifdef B2
#undef B2
#endif
#ifdef B3
#undef B3
#endif

#if defined(__GNUC__) || defined(__clang__)
    #pragma push_macro("F")
    #undef F
    #include <Eigen/Dense>
    #pragma pop_macro("F")
#else
    // conservative fallback: undef F around Eigen include
    #ifdef F
        #undef F
    #endif
    #include <Eigen/Dense>
#endif
#include <vector>

enum EndEffector{
    FLANGE,
    NEEDLE_INSERTION, // Tip fully inserted at (220, 0, 43). Retracted, it would be (100, 0, 43).
    CALIBRATION,
    // Add more end-effectors here
    COUNT // For counting the number of end-effectors, always keep at the end
};

// Changed pi to more modern C++ variant and removed the need for casting by defining it as a float
constexpr float PI_FLOAT = 3.14159265358979323846f;

class KinematicsController
{
public:
    // Set default constructor and deconstructor, required for inherited class
protected:
    KinematicsController() = default;

public:    
    virtual ~KinematicsController() = default;

    // Make some shorter type names
    using Matrix4f = Eigen::Matrix4f;
    using Matrix3f = Eigen::Matrix3f;
    using Vector3f = Eigen::Vector3f;
    using Vector6f = Eigen::Matrix<float, 6, 1>;

    // FK:
    // get end effector pose, based on joint steps
    Matrix4f forwardKinematicsEE(const std::vector<int> &jointSteps) const;

    // FK:
    // get end effector pose, as well as each joint pose, based on joint steps
    std::vector<Matrix4f> forwardKinematicsAll(const std::vector<int> &jointSteps) const;

    // IK:
    // input desired end-effector pose, output all possible joint steps (could be multiple solutions)
    virtual std::vector<std::vector<int>> inverseKinematics(const Matrix4f &H_desired) const = 0;

    // Filter:
    // This selects one of the solutions returned by inverseKinematics, based on distance
    // For obstavle avoidance, we want to know the phyical dimentions of the robot, this is a simpler implementation.
    std::vector<int> pickClosestSolution(const std::vector<std::vector<int>> &possible, const std::vector<int> &current) const;

    // End effector setting, set in inherited class as this is robot specific.
    // Method is made empty to avoid forcing all inherited classes to implement it.
    virtual void setEndEffector(EndEffector ee) {}
    EndEffector getEndEffector() const {return currentEE_;}

    // Convert a 6-element target vector [x, y, z, pitch, yaw, roll]
    // into a homogeneous 4x4 matrix (base -> end-effector).
    virtual Matrix4f targetToHomogeneous(const std::vector<float> &target) const;

    // Convert homogenous 4x4 matrix to pose with euler angles [x, y, z, pitch, yaw, roll]
    std::vector<float> homogenousToPose(const Matrix4f &H) const;

    // steps ↔ radians conversion helpers
    std::vector<float> stepsToRad(const std::vector<int> &steps) const;
    std::vector<int> radToSteps(const std::vector<float> &jointAngles) const;

protected: 
    // Helper functions

    // Product of exponentials helpers
    // convert from vector to tilde matrix
    Matrix4f twistToTilde(const Vector6f &twist) const;                        // Computes the tilde form of a twist
    Matrix4f matrixExponential(const Matrix4f &tildeTwist, float theta) const; // Computes the matrix exponential of a twist using Rodrigues' formula

    // Private variables
    std::size_t dof_ = 0;

    // Zero-configuration transforms
    std::vector<Matrix4f> H_joints0_;       // Homogeneous transforms of joints at zero config
    Matrix4f H_ee0_ = Matrix4f::Identity(); // Homogeneous transform of end-effector at zero config

    // Current end-effector
    EndEffector currentEE_ = EndEffector::FLANGE;

    // Kinematic data
    std::vector<Vector6f> twists_;      // All twists of the robot
    std::vector<Matrix4f> tildeTwists_; // Tilde forms of the twists

    // Drive data
    std::vector<int> stepsPerRev_; // Revolutions per revolution per joint
};



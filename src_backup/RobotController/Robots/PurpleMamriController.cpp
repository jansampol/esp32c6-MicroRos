template <typename T>
static T clampValue(T v, T lo, T hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

#include <algorithm>

#include "RobotController/Robots/PurpleMamriController.h"

const float PI_2 = 0.5f * PI_FLOAT;

void PurpleMamriController::setupGeometry() {
    H_joints0_.clear();
    twists_.clear();

    // --- 1. Set the steps per revolution for each joint ---
    stepsPerRev_ = std::vector<int>{5184, 3240, 1620, 1620, 2304};

    // --- 2. Define H matrices at zero configuration ---
    Matrix4f H100, H210, H320, H430, H540;

    H100 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 71,
        0, 0, 0, 1;

    H210 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 71,
        0, 0, 0, 1;

    H320 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 221,
        0, 0, 0, 1;

    H430 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 371,
        0, 0, 0, 1;

    H540 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 371,
        0, 0, 0, 1;

    H_joints0_ = {H100, H210, H320, H430, H540};

    // --- 3. Define twists (in space frame) ---
    Vector6f T100, T210, T320, T430, T540;

    T100 << 0, 0, 1, 0, 0, 0;
    T210 << 0, 1, 0, -71, 0, 0;
    T320 << 0, 1, 0, -221, 0, 0;
    T430 << 0, 1, 0, -371, 0, 0;
    T540 << 0, 0, 1, 0, 0, 0;

    twists_ = {T100, T210, T320, T430, T540};

    // --- 4. Set end-effector zero config ---
    setEndEffector(EndEffector::FLANGE);
}

std::vector<std::vector<int>>
PurpleMamriController::inverseKinematics(const Matrix4f &H_desired) const
{
    std::vector<std::vector<int>> allSolutions;

    if (dof_ != 5)
    {
        // This analytic IK is only implemented for a 5-DOF arm
        return allSolutions;
    }

    using Vector3f = KinematicsController::Vector3f;
    using Matrix3f = KinematicsController::Matrix3f;
    using Matrix4f = KinematicsController::Matrix4f;

    // ------------------------------------------------------------
    // 1. Convert desired EE pose to wrist (joint 5) pose H_0,5^d
    // ------------------------------------------------------------
    //
    // At zero config:
    //   H_ee0_ = H_0,5(0) * H_5,EE
    // => H_5,EE^{-1} = H_ee0_^{-1} * H_0,5(0)
    //
    Matrix4f H5_EE_inv = H_ee0_.inverse() * H_joints0_.back();
    Matrix4f H50d = H_desired * H5_EE_inv; // desired pose of joint-5 frame

    // Wrist center position
    float x5 = H50d(0, 3);
    float y5 = H50d(1, 3);
    float z5 = H50d(2, 3);

    // ------------------------------------------------------------
    // 2. Basic geometry: base height and link lengths
    // ------------------------------------------------------------
    // Base/shoulder height (z of first joint frame at zero)
    float z1 = H_joints0_[0](2, 3);

    // Link lengths along z between joints:
    float z2 = H_joints0_[1](2, 3);
    float z3 = H_joints0_[2](2, 3);
    float z4 = H_joints0_[3](2, 3);
    float L1 = std::fabs(z3 - z2);
    float L2 = std::fabs(z4 - z3);

    // Distance from shoulder to wrist center
    Vector3f p_shoulder = H_joints0_[0].block<3, 1>(0, 3);
    float r1 = (Vector3f(x5, y5, z5) - p_shoulder).norm();

    // θ₁ (shoulder yaw), primary solution
    float th1_1 = std::atan2(y5, x5);

    // ------------------------------------------------------------
    // Helper: wrap angle into [-π, π]
    // ------------------------------------------------------------
    auto wrapToPi = [](float angle) -> float
    {
        const float twoPI = 2.0f * PI_FLOAT;
        float a = std::fmod(angle, twoPI);
        if (a <= -PI_FLOAT)
            a += twoPI;
        else if (a > PI_FLOAT)
            a -= twoPI;
        if (std::fabs(a + PI_FLOAT) < 1e-6f)
            a = PI_FLOAT;
        return a;
    };

    // ------------------------------------------------------------
    // Helper: wrist angles (θ₄, θ₅) given (θ₁, θ₂, θ₃)
    // ------------------------------------------------------------
    auto wristAngles = [&](float t1, float t2, float t3,
                           float &t4, float &t5)
    {
        // Calculate forward kinematics using Brockett's exponential formula
        // to frame 3
        Matrix4f H03 = Matrix4f::Identity();
        H03 = H03 * matrixExponential(tildeTwists_[0], t1);
        H03 = H03 * matrixExponential(tildeTwists_[1], t2);
        H03 = H03 * matrixExponential(tildeTwists_[2], t3);
        H03 = H03 * H_joints0_[2];

        // Calculate relative rotation from frame 3 to 5 R35
        Matrix3f R03 = H03.block<3, 3>(0, 0);
        Matrix3f R05 = H50d.block<3, 3>(0, 0);
        Matrix3f R35 = R03.transpose() * R05;

        // R35 is a product of two pure rotations of th3 and th4
        // R35(0, 2) = sin(th4), R35(2, 2) = cos(th4)
        // tan(th4) = sin(th4) / cos(th4)
        // --> th4 = atan2(sin(th4), cos(th4))
        t4 = std::atan2(R35(0, 2), R35(2, 2));

        // R35(1, 0) = sin(th5), R35(1, 1) = cos(th5)
        t5 = std::atan2(R35(1, 0), R35(1, 1));
    };

    // ------------------------------------------------------------
    // 3. Law of cosines for elbow geometry (θ₂, θ₃)
    // ------------------------------------------------------------
    float cos2 = (L2 * L2 - L1 * L1 - r1 * r1) / (-2.0f * L1 * r1);
    float cos3 = (r1 * r1 - L2 * L2 - L1 * L1) / (-2.0f * L2 * L1);
    cos2 = clampValue(cos2, -1.0f, 1.0f);
    cos3 = clampValue(cos3, -1.0f, 1.0f);

    // ------------------------------------------------------------
    // 4. Enumerate 4 configurations:
    //    - 2 options for shoulder flip
    //    - 2 options for elbow up and down
    //      2 * 2 = 4 total configurations
    // ------------------------------------------------------------
    for (int mode = 0; mode < 4; ++mode)
    {
        float th1, th2, th3;

        // Elbow up and down modes, flips on odd and even modes
        bool elbowUp = (mode % 2) < 1;
        float sign = elbowUp ? -1.0f : +1.0f;

        float acos2 = std::acos(cos2);
        float acos3 = std::acos(cos3);

        // Shoulder flip mode
        if (mode < 2)
        {
            // Primary shoulder configuration
            th1 = th1_1;
            th2 = PI_2 - std::atan2(z5 - z1, std::hypot(x5, y5)) + sign * acos2;
            float th3_raw = PI_FLOAT - acos3;
            th3 = elbowUp ? th3_raw : -th3_raw;
        }
        else
        {
            // Shoulder flipped by π
            th1 = wrapToPi(th1_1 + PI_FLOAT);
            th2 = -(PI_2 - std::atan2(z5 - z1, std::hypot(x5, y5)) + sign * acos2);
            float th3_raw = PI_FLOAT - acos3;
            th3 = elbowUp ? -th3_raw : th3_raw;
        }

        float th4, th5;
        wristAngles(th1, th2, th3, th4, th5);

        std::vector<float> q_rad(5);
        q_rad[0] = th1;
        q_rad[1] = th2;
        q_rad[2] = th3;
        q_rad[3] = th4;
        q_rad[4] = th5;

        allSolutions.push_back(radToSteps(q_rad));
    }

    return allSolutions;
}


void PurpleMamriController::setEndEffector(EndEffector ee) {
    Matrix4f H_EE = Eigen::Matrix4f::Identity();

    switch (ee) {
        // Default to flange if invalid end-effector type is provided
        default:
        case EndEffector::FLANGE:
            H_EE(2, 3) = 86.0f; // +z
            currentEE_ = EndEffector::FLANGE;
            break;
        case EndEffector::CALIBRATION:
            H_EE(0, 3) = 168.0f; // +x
            H_EE(2, 3) = 86.0f + 25.0f; // +z
            currentEE_ = EndEffector::CALIBRATION;
            break;
        case EndEffector::NEEDLE_INSERTION:
            H_EE(0, 3) = 220.0f; // +x (maximum insertion depth)
            H_EE(2, 3) = 86.0f + 43.0f; // +z (flange height + needle height)
            currentEE_ = EndEffector::NEEDLE_INSERTION;
            break;
    }

    H_ee0_ = H_joints0_.back() * H_EE;
}


// Convert target vector [x, y, z, pitch, yaw, roll] to homogeneous matrix
// For the purple MAMRI this is interpreted as:
// x, y, z: position of end-effector
// pitch: pitch of the end-effector, relative to the rotation of the base (shoulder joint 1)
// yaw: rotation of the flange (joint 5)
// roll: ignored as the robot has only 5 DOF and cannot control all 3 rotational DOF
Eigen::Matrix4f PurpleMamriController::targetToHomogeneous(const std::vector<float> &target) const
{
    Matrix4f H = Matrix4f::Identity();

    float x = target[0];
    float y = target[1];
    float z = target[2];
    float arm_pitch = target[3]; 
    float flange_spin = target[4];

    // In order to calculate the correct base rotation, we need to do some math
    // to account for the fact that pitching the arm and rotating the flange
    // will cause the tool tip to "swing" in an arc. The base rotation needs to
    // create a correction angle that aligns the swing arc with the target position.
    // 
    // Overview of the calculations:
    // 1. Get the tool transformation matrix relative to the wrist frame at zero config
    // 2. Extract the tool's translation vector in the wrist frame
    // 3. Calculate virtual translation of the tool tip if we apply the desired pitch and flange spin, without base rotation
    // 4. Calculate base yaw angle that would align the tool swing with the target position
    // 5. Construct final homogeneous matrix with the calculated base rotation, desired pitch and flange

    // 1. Get the tool transformation matrix relative to the wrist frame at zero config
    Matrix4f H_0_5 = H_joints0_.back();
    Matrix4f H_tool = H_0_5.inverse() * H_ee0_;
    
    // 2. Extract the tool's translation vector in the wrist frame
    Eigen::Vector3f t_tool = H_tool.block<3, 1>(0, 3);

    // 3. Calculate virtual translation of the tool tip if we apply the desired pitch and flange spin, without base rotation
    Eigen::AngleAxisf Ry_pitch(arm_pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf Rz_flange(flange_spin, Eigen::Vector3f::UnitZ());
    Eigen::Vector3f t_virtual = Ry_pitch * Rz_flange * t_tool;

    // 4. Calculate base yaw angle that would align the tool swing with the target position
    //
    // The Y-component of t_virtual is the "tangential offset" (swing)
    // that the base rotation needs to account for. Imagine dist_xy as
    // the hypotenuse of a right triangle in the xy plane of the world
    // frame. v_y is the opposite side while the adjacent side is the
    // "effective reach" of the arm after pitching and flange rotation.
    // The base rotation needs to create a correction angle that aligns 
    // the effective reach with the target position.
    float v_y = t_virtual.y();
    float dist_xy = std::hypot(x, y);
    float correction = 0.0f;
    if (dist_xy > std::abs(v_y)) {
        correction = std::asin(v_y / dist_xy);
    } else {
        // Target is unreachable (inside the swing radius)
        correction = (v_y > 0) ? PI_2 : -PI_2;
    }
    
    float uncorrected_yaw_angle = std::atan2(y, x);
    float real_base_yaw = uncorrected_yaw_angle - correction;

    // 5. Construct final homogeneous matrix

    // Calculate final rotation matrix
    // Final Rotation = Base * Pitch * Flange
    // Order: Z-Y-Z
    Eigen::AngleAxisf Rz_base(real_base_yaw, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f R = (Rz_base * Ry_pitch * Rz_flange).toRotationMatrix();
    H.block<3,3>(0,0) = R;
    H.block<3,1>(0,3) = Eigen::Vector3f(x, y, z);
    
    return H;
}
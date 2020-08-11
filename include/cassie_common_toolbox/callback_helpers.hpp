/*
 * @brief Helper functions for unpacking messages.
 * @author Jenna Reher (jreher@caltech.edu)
 */

#ifndef CASSIE_CALLBACK_HELPERS_HPP
#define CASSIE_CALLBACK_HELPERS_HPP

#include <Eigen/Dense>
#include <cassie_common_toolbox/geometry.hpp>
#include <cassie_common_toolbox/CassieStateEnum.hpp>
#include <cassie_common_toolbox/cassie_control_msg.h>
#include <cassie_common_toolbox/cassie_estimation_msg.h>
#include <cassie_common_toolbox/cassie_proprioception_msg.h>

void unpack_proprioception(const cassie_common_toolbox::cassie_proprioception_msg::ConstPtr& propmsg, Eigen::VectorXd &q, Eigen::VectorXd &dq, Eigen::VectorXd &radio, Eigen::Vector3d &gyro, Eigen::Vector3d &accel, Eigen::VectorXd &torque, double &leftContact, double &rightContact) {
    for (unsigned int i=0; i<16; ++i)
        radio[i] = propmsg->radio[i];

    q(LeftHipRoll)      = propmsg->encoder_position[0];
    q(LeftHipYaw)       = propmsg->encoder_position[1];
    q(LeftHipPitch)     = propmsg->encoder_position[2];
    q(LeftKneePitch)    = propmsg->encoder_position[3];
    q(LeftShinPitch)    = propmsg->encoder_position[4];
    q(LeftTarsusPitch)  = propmsg->encoder_position[5];
    q(LeftFootPitch)     = propmsg->encoder_position[6];
    q(RightHipRoll)     = propmsg->encoder_position[7];
    q(RightHipYaw)      = propmsg->encoder_position[8];
    q(RightHipPitch)    = propmsg->encoder_position[9];
    q(RightKneePitch)   = propmsg->encoder_position[10];
    q(RightShinPitch)   = propmsg->encoder_position[11];
    q(RightTarsusPitch) = propmsg->encoder_position[12];
    q(RightFootPitch)    = propmsg->encoder_position[13];

    q(LeftHeelSpring)   = propmsg->q_achilles[0];
    q(RightHeelSpring)  = propmsg->q_achilles[1];

    // Get encoder velocities
    dq(LeftHipRoll)      = propmsg->encoder_velocity[0];
    dq(LeftHipYaw)       = propmsg->encoder_velocity[1];
    dq(LeftHipPitch)     = propmsg->encoder_velocity[2];
    dq(LeftKneePitch)    = propmsg->encoder_velocity[3];
    dq(LeftShinPitch)    = propmsg->encoder_velocity[4];
    dq(LeftTarsusPitch)  = propmsg->encoder_velocity[5];
    dq(LeftFootPitch)     = propmsg->encoder_velocity[6];
    dq(RightHipRoll)     = propmsg->encoder_velocity[7];
    dq(RightHipYaw)      = propmsg->encoder_velocity[8];
    dq(RightHipPitch)    = propmsg->encoder_velocity[9];
    dq(RightKneePitch)   = propmsg->encoder_velocity[10];
    dq(RightShinPitch)   = propmsg->encoder_velocity[11];
    dq(RightTarsusPitch) = propmsg->encoder_velocity[12];
    dq(RightFootPitch)    = propmsg->encoder_velocity[13];

    dq(LeftHeelSpring) = propmsg->dq_achilles[0];
    dq(RightHeelSpring) = propmsg->dq_achilles[1];

    // Assign gyroscope
    dq(3) = propmsg->angular_velocity.x;
    dq(4) = propmsg->angular_velocity.y;
    dq(5) = propmsg->angular_velocity.z;
    gyro(0) = propmsg->angular_velocity.x;
    gyro(1) = propmsg->angular_velocity.y;
    gyro(2) = propmsg->angular_velocity.z;

    // Assign accelerometer
    accel(0) = propmsg->linear_acceleration.x;
    accel(1) = propmsg->linear_acceleration.y;
    accel(2) = propmsg->linear_acceleration.z;

    // Get contacts
    leftContact = propmsg->contact[0];
    rightContact = propmsg->contact[1];

    // Get joint torques
    for (unsigned int i=0; i<10; ++i)
        torque(i) = propmsg->motor_torque[i];

}

void get_proprioception_encoders(const cassie_common_toolbox::cassie_proprioception_msg& propmsg, Eigen::VectorXd &q, Eigen::VectorXd &dq) {
    q(LeftHipRoll)      = propmsg.encoder_position[0];
    q(LeftHipYaw)       = propmsg.encoder_position[1];
    q(LeftHipPitch)     = propmsg.encoder_position[2];
    q(LeftKneePitch)    = propmsg.encoder_position[3];
    q(LeftShinPitch)    = propmsg.encoder_position[4];
    q(LeftTarsusPitch)  = propmsg.encoder_position[5];
    q(LeftFootPitch)     = propmsg.encoder_position[6];
    q(RightHipRoll)     = propmsg.encoder_position[7];
    q(RightHipYaw)      = propmsg.encoder_position[8];
    q(RightHipPitch)    = propmsg.encoder_position[9];
    q(RightKneePitch)   = propmsg.encoder_position[10];
    q(RightShinPitch)   = propmsg.encoder_position[11];
    q(RightTarsusPitch) = propmsg.encoder_position[12];
    q(RightFootPitch)    = propmsg.encoder_position[13];

    // Get encoder velocities
    dq(LeftHipRoll)      = propmsg.encoder_velocity[0];
    dq(LeftHipYaw)       = propmsg.encoder_velocity[1];
    dq(LeftHipPitch)     = propmsg.encoder_velocity[2];
    dq(LeftKneePitch)    = propmsg.encoder_velocity[3];
    dq(LeftShinPitch)    = propmsg.encoder_velocity[4];
    dq(LeftTarsusPitch)  = propmsg.encoder_velocity[5];
    dq(LeftFootPitch)     = propmsg.encoder_velocity[6];
    dq(RightHipRoll)     = propmsg.encoder_velocity[7];
    dq(RightHipYaw)      = propmsg.encoder_velocity[8];
    dq(RightHipPitch)    = propmsg.encoder_velocity[9];
    dq(RightKneePitch)   = propmsg.encoder_velocity[10];
    dq(RightShinPitch)   = propmsg.encoder_velocity[11];
    dq(RightTarsusPitch) = propmsg.encoder_velocity[12];
    dq(RightFootPitch)    = propmsg.encoder_velocity[13];
}

void get_proprioception_orientation(const cassie_common_toolbox::cassie_proprioception_msg& propmsg, Eigen::VectorXd &q, Eigen::VectorXd &dq, Eigen::Quaterniond &quat) {
    // Get the pelvis rotation via the IMU
    // Assign the pelvis rotation
    quat.w() = propmsg.orientation.w;
    quat.x() = propmsg.orientation.x;
    quat.y() = propmsg.orientation.y;
    quat.z() = propmsg.orientation.z;

    Eigen::Matrix3d RateMatrix;
    //RateMatrix <<
    //    1., sin(q(BaseRotX))*tan(q(BaseRotY)), cos(q(BaseRotX))*tan(q(BaseRotY)),
    //    0, cos(q(BaseRotX)), -sin(q(BaseRotX)),
    //    0, sin(q(BaseRotX))/cos(q(BaseRotY)), cos(q(BaseRotX))/cos(q(BaseRotY));
    RateMatrix <<
        1.0/cos(q(BaseRotY)), 0.0, 0.0,
        0,                    1.0, 0.0,
        tan(q(BaseRotY)),     0.0, 1.0;

    dq(BaseRotX) = propmsg.angular_velocity.x;
    dq(BaseRotY) = propmsg.angular_velocity.y;
    dq(BaseRotZ) = propmsg.angular_velocity.z;
    dq.segment(BaseRotX,3) = RateMatrix * dq.segment(BaseRotX,3);

    // Do euler angles - SUBTRACTING OUT THE YAW!!!
    Eigen::Matrix3d R = quat.toRotationMatrix();
    Eigen::EulerAnglesXYZd euler = Eigen::EulerAnglesXYZd::FromRotation<false, false, false>(quat);
    eulerXYZ(quat, euler);
    Eigen::Matrix3d Rz;
    Rz << cos(euler.gamma()), -sin(euler.gamma()), 0,
          sin(euler.gamma()), cos(euler.gamma()),  0,
          0,                  0,                   1;
    R = Rz.transpose() * R;
    Eigen::Quaterniond tempquat(R);
    eulerXYZ(tempquat, euler);
    q(BaseRotX) = euler.alpha(); // roll
    q(BaseRotY) = euler.beta();  // pitch
    q(BaseRotZ) = euler.gamma(); // yaw

    // Linear velocity
    dq(BasePosX) = propmsg.linear_velocity.x;
    dq(BasePosY) = propmsg.linear_velocity.y;
    dq(BasePosZ) = propmsg.linear_velocity.z;
}

void unpack_estimation(const cassie_common_toolbox::cassie_estimation_msg::ConstPtr& estmsg, Eigen::VectorXd &q, Eigen::VectorXd &dq, Eigen::Quaterniond &quat) {
    q(BasePosX) = estmsg->pose.position.x;
    q(BasePosY) = estmsg->pose.position.y;
    q(BasePosZ) = estmsg->pose.position.z;

    quat.w() = estmsg->pose.orientation.w;
    quat.x() = estmsg->pose.orientation.x;
    quat.y() = estmsg->pose.orientation.y;
    quat.z() = estmsg->pose.orientation.z;

    // Do euler angles - SUBTRACTING OUT THE YAW!!!
    Eigen::Matrix3d R = quat.toRotationMatrix();
    Eigen::EulerAnglesXYZd euler = Eigen::EulerAnglesXYZd::FromRotation<false, false, false>(quat);
    eulerXYZ(quat, euler);
    Eigen::Matrix3d Rz;
    Rz << cos(euler.gamma()), -sin(euler.gamma()), 0,
          sin(euler.gamma()), cos(euler.gamma()),  0,
          0,                  0,                   1;
    R = Rz.transpose() * R;
    Eigen::Quaterniond tempquat(R);
    eulerXYZ(tempquat, euler);
    q(BaseRotX) = euler.alpha(); // roll
    q(BaseRotY) = euler.beta();  // pitch
    q(BaseRotZ) = euler.gamma(); // yaw

    // Velocity is forward facing (remove yaw)
    //dq(BasePosX) = estmsg->twist.linear.x;
    //dq(BasePosY) = estmsg->twist.linear.y;
    //dq(BasePosZ) = estmsg->twist.linear.z;
    //dq.block(BasePosX,0,3,1) = Rz.transpose() * dq.block(BasePosX,0,3,1);
}


#endif // CASSIE_CALLBACK_HELPERS_HPP

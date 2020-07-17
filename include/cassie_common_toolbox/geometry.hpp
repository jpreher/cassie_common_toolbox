/*
 * @author Jenna Reher (jreher@caltech.edu)
 */

#ifndef CASSIE_GEOMETRY_HPP
#define CASSIE_GEOMETRY_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>

/* FUNCTION eulerZYX(float q[4], float euler[3])
 * Convert the quaternion to euler angles in ZYX rotation order.
 */
void eulerZYX(Eigen::Quaterniond &q, Eigen::EulerAnglesZYXd &euler);

void eulerXYZ(Eigen::Quaterniond &q, Eigen::EulerAnglesXYZd &euler);

Eigen::Matrix3d skew(Eigen::Vector3d &v);


#endif // CASSIE_GEOMETRY_HPP

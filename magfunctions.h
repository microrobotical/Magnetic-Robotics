#ifndef MAGFUNCTIONS_H
#define MAGFUNCTIONS_H

#define _USE_MATH_DEFINES
#include <math.h>
#include <Eigen/Eigen>

namespace magF
{
// Note M_PI is accurate to 16 digits after the decimal place
const double mu0 = M_PI * 4e-7;

Eigen::Vector3d calc_dipole_field(Eigen::Vector3d dipoleMoment, Eigen::Vector3d relPosition);
//Eigen::Matrix<double,6,1> wrenchOnDipole(Eigen::Vector3d dipoleMoment, Eigen::Vector3d position, Eigen::Vector3d field, Eigen::Matrix<double,5,1> gradient);
Eigen::Matrix<double,6,1> calc_wrench_on_dipole(Eigen::Vector3d dipoleMoment, Eigen::Vector3d position, Eigen::Matrix<double,8,1> augField);
//Eigen::Vector3d calc_torque_on_dipole(Eigen::Vector3d dipoleMoment, Eigen::Vector3d field);
//Eigen::Vector3d calc_force_on_dipole(Eigen::Vector3d dipoleMoment, Eigen::Matrix<double,5,1> gradient);
Eigen::Matrix3d calc_dipole_torque_matrix(Eigen::Vector3d dipoleMoment);
Eigen::Matrix<double,3,5> calc_dipole_force_matrix(Eigen::Vector3d dipoleMoment);
Eigen::Matrix<double,6,8> calc_dipole_wrench_matrix(Eigen::Vector3d dipoleMoment, Eigen::Vector3d position);
Eigen::Matrix3d calc_xprod_skew_mat(Eigen::Vector3d v);
}
#endif // MAGFUNCTIONS_H

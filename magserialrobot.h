#ifndef MAGSERIALROBOT_H
#define MAGSERIALROBOT_H
#include "robotfunctions.h"
#include "magfunctions.h"
#include <iostream>

class MagSerialRobot : public robF::SerialRobot
{
public:
    MagSerialRobot(int numLinks);
    MagSerialRobot(int numLinks, double linkLength[], double linkTwist[],
                   double linkOffset[], double jointAngle[], int jointType[],
                   Eigen::Vector3d magnetLocal[],
                   Eigen::Vector3d magnetPosLocal[]);
    ~MagSerialRobot();
    Eigen::Vector3d m_get_magnet(int linkNumber);
    Eigen::Vector3d m_get_magnet_pos(int linkNumber);
    Eigen::Matrix<double, Eigen::Dynamic, 8> m_calc_magnetization_matrix();
    Eigen::MatrixXd m_calc_actuation_matrix();
    Eigen::Matrix<double, Eigen::Dynamic, 1> m_calc_applied_gen_forces(Eigen::Matrix<double, 8, 1> augField);
    Eigen::Matrix<double, Eigen::Dynamic, 1> m_calc_internal_gen_forces();
    void m_change_magnets(Eigen::Vector3d magnetLocal[],
                       Eigen::Vector3d magnetPosLocal[]);
    void m_set_coil_matrix(Eigen::Matrix<double, 8, Eigen::Dynamic>  coilMatrix);
    double m_SSV(void);
    double m_calc_min_ssv(Eigen::VectorXd qRes);
private:
    Eigen::Vector3d *mMagnetLocal;
    Eigen::Vector3d *mMagnetPosLocal;
    Eigen::Matrix<double,1,8> m_actuation_vec(int jointNumber);
    Eigen::JacobiSVD<Eigen::MatrixXd> mSVD;
    Eigen::VectorXd mSingularValues;
    Eigen::Matrix<double, 8, Eigen::Dynamic> mCoilMatrix;
    void m_calc_ssv(double& minSSV, Eigen::VectorXd& qMinSSV, Eigen::VectorXd qGridPoints[]);
    void m_calc_ssv(double& minSSV, Eigen::VectorXd& qMinSSV, Eigen::VectorXd qGridPoints[], int jointNum);
};

#endif // MAGSERIALROBOT_H

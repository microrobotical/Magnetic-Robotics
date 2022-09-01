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
    Eigen::Matrix<double, Eigen::Dynamic, 8> m_get_actuation_matrix();
    void m_set_magnets(Eigen::Vector3d magnetLocal[],
                       Eigen::Vector3d magnetPosLocal[]);
private:
    Eigen::Vector3d *mMagnetLocal;
    Eigen::Vector3d *mMagnetPosLocal;
    Eigen::Matrix<double,1,8> m_actuation_vec(int jointNumber);
};

#endif // MAGSERIALROBOT_H

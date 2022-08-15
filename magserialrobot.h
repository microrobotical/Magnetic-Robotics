#ifndef MAGSERIALROBOT_H
#define MAGSERIALROBOT_H
#include "robotfunctions.h"
#include "magfunctions.h"

class magSerialRobot : public robF::serialRobot
{
public:
    magSerialRobot(int numLinks, double linkLength[], double linkTwist[], double linkOffset[], double jointAngle[], int jointType[], Eigen::Vector3d magnetLocal[], Eigen::Vector3d magnetPosLocal[]);
    ~magSerialRobot();
    Eigen::Vector3d magnet(int linkNumber);
    Eigen::Vector3d magnetPos(int linkNumber);
    Eigen::Matrix<double, Eigen::Dynamic, 8> actuationMatrix();
private:
    Eigen::Vector3d* magnetLocal;
    Eigen::Vector3d* magnetPosLocal;
    Eigen::Matrix<double,1,8> actuationVec(int jointNumber);
};

#endif // MAGSERIALROBOT_H

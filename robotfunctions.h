#ifndef ROBOTFUNCTIONS_H
#define ROBOTFUNCTIONS_H

#include <Eigen/Eigen>

#define JOINTREV 0
#define JOINTPRS 1

namespace robF
{
Eigen::Matrix4d transformMatDH(int fromFrame, int toFrame, int numLinks, double linkLength[], double linkTwist[], double linkOffset[], double jointAngle[]);
Eigen::Matrix4d transformMatDH(double linkLength, double linkTwist, double linkOffset, double jointAngle);
Eigen::Matrix<double,6,1> changeScrewOrder(Eigen::Matrix<double,6,1> screw);

class serialRobot
{
public:
    serialRobot(int numLinks, double linkLength[], double linkTwist[], double linkOffset[], double jointAngle[], int jointType[]);
    serialRobot(int numLinks);
    ~serialRobot();
    Eigen::Matrix4d transformMat(int fromFrame, int toFrame);
    Eigen::Matrix4d transformMat();
    Eigen::Matrix<double,6,1> unitTwist(int jointNumber);
    double applyWrenchToJoint(int jointNumber, Eigen::Matrix<double,6,1> wrench);
    void setq(Eigen::VectorXd q);
    void setq(int jointNumber, double q);
    Eigen::VectorXd getq(void);
protected:
    int numLinks;
    double* linkLength;
    double* linkTwist;
    double* linkOffset;
    double* jointAngle;
    int* jointType;
};
}
#endif // ROBOTFUNCTIONS_H

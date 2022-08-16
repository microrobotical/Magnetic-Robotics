#ifndef ROBOTFUNCTIONS_H
#define ROBOTFUNCTIONS_H

#include <Eigen/Eigen>

#define JOINTREV 0
#define JOINTPRS 1

namespace robF
{
Eigen::Matrix<double,6,1> changeScrewOrder(Eigen::Matrix<double,6,1> screw);

class serialRobot
{
public:
    serialRobot(int numLinks, double linkLength[], double linkTwist[], double linkOffset[], double jointAngle[], int jointType[]);
    serialRobot(int numLinks);
    ~serialRobot();
    Eigen::Matrix4d transformMatSingleLink(int fromFrame);
    Eigen::Matrix4d transformMat(int fromFrame, int toFrame);
    Eigen::Matrix4d transformMat(int fromFrame);
    Eigen::Matrix4d transformMat();
    Eigen::Matrix4d transformMatGlobal(int fromFrame);
    Eigen::Matrix<double,6,1> unitTwist(int jointNumber);
    Eigen::Matrix<double,6,1> unitTwistGlobal(int jointNumber);
    double applyWrenchToJoint(int jointNumber, Eigen::Matrix<double,6,1> wrench);
    void setq(Eigen::VectorXd q);
    void setq(int jointNumber, double q);
    void setTbase(Eigen::Matrix4d T);
    Eigen::VectorXd getq(void);
protected:
    int numLinks;
    double* linkLength;
    double* linkTwist;
    double* linkOffset;
    double* jointAngle;
    int* jointType;
    Eigen::Matrix4d Tbase = Eigen::Matrix4d::Identity();
};
}
#endif // ROBOTFUNCTIONS_H

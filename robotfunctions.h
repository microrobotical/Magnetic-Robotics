#ifndef ROBOTFUNCTIONS_H
#define ROBOTFUNCTIONS_H
#define _USE_MATH_DEFINES
#include <math.h>
#include <Eigen/Eigen>

namespace robF
{
const int jointRev = 0;
const int jointPrs = 1;
const Eigen::Vector2d rangeRevDefault {-M_PI, M_PI};
const Eigen::Vector2d rangePrsDefault {-1000.0, 1000.0};
Eigen::Matrix<double,6,1> change_screw_order(Eigen::Matrix<double,6,1> screw);

class SerialRobot
{
public:
    SerialRobot();
    SerialRobot(int numLinks, double linkLength[], double linkTwist[], double linkOffset[], double jointAngle[], int jointType[]);
    SerialRobot(int numLinks);
    ~SerialRobot();
    void m_change_DH_params(double linkLength[], double linkTwist[], double linkOffset[], double jointAngle[], int jointType[]);
    Eigen::Matrix4d m_calc_transform_mat_single_link(int fromFrame);
    Eigen::Matrix4d m_calc_transform_mat(int fromFrame, int toFrame);
    Eigen::Matrix4d m_calc_transform_mat(int fromFrame);
    Eigen::Matrix4d m_calc_transform_mat();
    Eigen::Matrix4d m_calc_transform_mat_global(int fromFrame);
    Eigen::Matrix<double,6,1> m_calc_unit_twist(int jointNumber);
    Eigen::Matrix<double,6,1> m_calc_unit_twist_global(int jointNumber);
    double m_calc_gen_force_from_wrench(int jointNumber, Eigen::Matrix<double,6,1> wrench);
    void m_set_q(Eigen::VectorXd q);
    void m_set_q(int jointNumber, double q);
    void m_set_qRange(Eigen::MatrixXd qRange);
    void m_set_qRange(int jointNumber, Eigen::Vector2d qRange);
    void m_set_Tbase(Eigen::Matrix4d T);
    void m_display_properties(void);
    Eigen::VectorXd m_get_q(void);
protected:
    int mNumLinks;
    double* mLinkLength;
    double* mLinkTwist;
    double* mLinkOffset;
    double* mJointAngle;
    int* mJointType;
    double* mQRange;
    Eigen::Matrix4d mTBase = Eigen::Matrix4d::Identity();
};
}
#endif // ROBOTFUNCTIONS_H

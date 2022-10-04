#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <Eigen/Eigen>
#include "robotfunctions.h"
#include "magfunctions.h"
#include "magserialrobot.h"

int main(void)
{
    // Base coordinates (transformation from Link 0 frame to global frame).
    Eigen::Matrix4d T  {{0, 0,-1, 0},
                        {1, 0, 0, 0},
                        {0,-1, 0, 0},
                        {0, 0, 0, 1}};
    // Robot Denavit-Hartenberg parameters
    int numLinks = 2;
    double linkLength[2] = {6.55e-3, 8.0e-3}; //meters
    double linkTwist[2] = {1.571, 0.0}; //radians
    double linkOffset[2] = {0.0, 0.0}; //meters
    double jointAngle[2] = {0.0, 0.0}; //radians
    int jointType[2] = {robF::jointRev, robF::jointRev}; //0 for revolute, 1 for prismatic
    // Magnet dipole vectors and positions in local link coordinates
    Eigen::Vector3d magnetLocal[3];
    Eigen::Vector3d magnetPosLocal[3];
    magnetLocal[0] << -2.681e-3, 0, 0; // A.m^2
    magnetLocal[1] << 35.859e-3, 0, 0; // A.m^2
    magnetLocal[2] << 0, -26.813e-3, 0; // A.m^2
    magnetPosLocal[0] << -3.2e-3, 0, 0; // m
    magnetPosLocal[1] << -3.5e-3, 0, 0; // m
    magnetPosLocal[2] << -4.25e-3, 0.9e-3, 0; // m
    // Joint limits
    Eigen::MatrixXd qRange(2,2);
    qRange << -85.0/180.0*M_PI, 85.0/180.0*M_PI, 0.0, M_PI/3; // rad
    // EM Coil system matrix [b;g] = L*u
    Eigen::Matrix<double,8,8> Mcoil;
    Mcoil <<  3.6,   -0.7,  -4.1, -17.2,   17.5,  -3.4,   1.7,   4.0, // mT/%A
          3.7,   18.1,   3.5,  -1.0,    0.8,  -4.0, -17.0,  -3.6, // mT/%A
         -0.7,   12.5,  -1.2,  12.2,   12.1,  -1.1,  12.1,  -1.1, // mT/%A
        -15.3,  153.5, -19.1, -79.5,  -93.5, -12.2, 154.8, -23.0, // mT/m.%A
        -38.3,    3.6,  41.2,  -6.3,   -0.8, -37.1,  15.7,  36.4, // mT/m.%A
         -8.3,   15.3,   9.9, 231.3, -227.4,   7.0, -11.5, -10.9, // mT/m.%A
        -18.7,  -90.1, -14.4, 149.2,  164.0, -20.3, -96.4, -13.9, // mT/m.%A
        -10.8, -247.7,  -9.4,   9.0,  -20.5,   9.7, 230.4,   8.7; // mT/m.%A
    Mcoil = Mcoil * 1e-3 / 24; // T/A and T/m.A
    // Instantiate a magSerialRobot object
    MagSerialRobot myMagbot(numLinks);
    // Set the Denavit-Hartenberg parameters (robot geometry/kinematics).
    myMagbot.m_change_DH_params(linkLength, linkTwist, linkOffset, jointAngle, jointType);
    // Set the magnetic properties of the links (dipole orientation and location
    // in local link frame coords).
    myMagbot.m_change_magnets(magnetLocal, magnetPosLocal);
    // Set the coil matrix.
    myMagbot.m_set_coil_matrix(Mcoil);
    // Set the joint limits.
    myMagbot.m_set_qRange(qRange);
    // Set the base to global transformation.
    myMagbot.m_set_Tbase(T);
    // Set the configuration of the robot (generalized coordinates)
    Eigen::Vector2d q {0.0, M_PI/3}; //[rad]
    myMagbot.m_set_q(q);
    // Display the final robot DH parameters in the terminal.
    myMagbot.m_display_properties();
    // Grid search resolution.
    Eigen::VectorXd qRes(numLinks);
    qRes << 1.0/180.0*M_PI, 1.0/180.0*M_PI;
    // Variable to store the min SSV.
    double minSSV; //N.m/A
    // Vector for storing the configuration that corresponds to the min SSV
    Eigen::Vector2d qMinSSV;
    // Find the minimum smallest singular value of the robot actuation matrix.
    minSSV = myMagbot.m_calc_min_ssv(qRes);
    qMinSSV = myMagbot.m_get_q();
    std::cout << "Minimum SSV: " << minSSV << " N.m/A at (" << qMinSSV(0) << ", " << qMinSSV(1) << ") rad" << std::endl;
}

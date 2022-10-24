#ifndef MAGSERIALROBOT_H
#define MAGSERIALROBOT_H
#include "robotfunctions.h"
#include "magfunctions.h"
#include <unsupported/Eigen/NonLinearOptimization>
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
    double m_calc_min_ssv();
    void m_optimize_dipole_vectors(void);
private:
    Eigen::Vector3d *mMagnetLocal;
    Eigen::Vector3d *mMagnetPosLocal;
    Eigen::Matrix<double,1,8> m_actuation_vec(int jointNumber);
    Eigen::JacobiSVD<Eigen::MatrixXd> mSVD;
    Eigen::VectorXd mSingularValues;
    Eigen::Matrix<double, 8, Eigen::Dynamic> mCoilMatrix;
    void m_calc_ssv(double& minSSV, Eigen::VectorXd& qMinSSV, Eigen::VectorXd qGridPoints[]);
    void m_calc_ssv(double& minSSV, Eigen::VectorXd& qMinSSV, Eigen::VectorXd qGridPoints[], int jointNum);
    struct LMfunctor
    {
        LMfunctor(MagSerialRobot* pMagBot)
        {
            mpMagBot = pMagBot;
        }
        // Number of data points
        int m;
        // Number of parameters
        int n;
        int values() const {return m;}
        int inputs() const {return n;}
        MagSerialRobot* mpMagBot;
        int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
        {
            Eigen::Vector3d *magnetLocal = new Eigen::Vector3d[mpMagBot->mNumLinks+1];
            double dipoleMag;
            double azimuthAngle;
            double polarAngle;
            double minSSV;
            magnetLocal[0] = mpMagBot->mMagnetLocal[0];
            for (int linkNum = 1; linkNum <= mpMagBot->mNumLinks; linkNum++)
            {
                dipoleMag = x(3*(linkNum-1));
                azimuthAngle = x(3*(linkNum-1)+1);
                polarAngle = x(3*(linkNum-1)+2);
                magnetLocal[linkNum] << dipoleMag*cos(azimuthAngle)*sin(polarAngle),
                                        dipoleMag*sin(azimuthAngle)*sin(polarAngle),
                                        dipoleMag*cos(polarAngle);
            }
            mpMagBot->m_change_magnets(magnetLocal, mpMagBot->mMagnetPosLocal);
            // Calculate the reciprocal of the minimum SSV with default grid
            // resolution.
            minSSV = mpMagBot->m_calc_min_ssv();
            std::cout << minSSV;
            fvec(0) = float(1.0/minSSV);
            return 0;
        }
        int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
        {
            // Calculate the Jacobian matrix numerically using the secant
            // method.
            int paramNum;
            float epsilonAngle = 1.0f/180.0f*float(M_PI); //rad
            float epsilonMag = 1e-4f; //A.m^2
            Eigen::VectorXf xPlus(inputs());
            Eigen::VectorXf xMinus(inputs());
            Eigen::VectorXf fvecPlus(values());
            Eigen::VectorXf fvecMinus(values());
            Eigen::VectorXf fvecDiff(values());
            for (int linkNum = 1; linkNum <= mpMagBot->mNumLinks; linkNum++)
            {
                // Rate of change wrt dipole magnitude
                paramNum = 3*(linkNum-1);
                xPlus = x;
                xMinus = x;
                xPlus(paramNum) += epsilonMag;
                xMinus(paramNum) -= epsilonMag;
                operator()(xPlus, fvecPlus);
                operator()(xMinus, fvecMinus);
                fvecDiff = (fvecPlus - fvecMinus) / (2.0 * epsilonMag);
                fjac.block(0, paramNum, values(), 1) = fvecDiff;
                // Rate of change wrt dipole azimuth angle
                paramNum = 3*(linkNum-1)+1;
                xPlus = x;
                xMinus = x;
                xPlus(paramNum) += epsilonAngle;
                xMinus(paramNum) -= epsilonAngle;
                operator()(xPlus, fvecPlus);
                operator()(xMinus, fvecMinus);
                fvecDiff = (fvecPlus - fvecMinus) / (2.0 * epsilonAngle);
                fjac.block(0, paramNum, values(), 1) = fvecDiff;
                // Rate of change wrt dipole polar angle
                paramNum = 3*(linkNum-1)+2;
                xPlus = x;
                xMinus = x;
                xPlus(paramNum) += epsilonAngle;
                xMinus(paramNum) -= epsilonAngle;
                operator()(xPlus, fvecPlus);
                operator()(xMinus, fvecMinus);
                fvecDiff = (fvecPlus - fvecMinus) / (2.0 * epsilonAngle);
                fjac.block(0, paramNum, values(), 1) = fvecDiff;
                std::cout << fjac << std::endl << std::endl;
            }
            return 0;
        }
    };
};

#endif // MAGSERIALROBOT_H

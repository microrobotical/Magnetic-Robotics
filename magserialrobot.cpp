#include "magserialrobot.h"

MagSerialRobot::MagSerialRobot(int numLinks) : robF::SerialRobot(numLinks)
{
    /* Simplified constructor for the MagSerialRobot class.
     *
     * Inputs:
     * int numLinks - The number of links in the robot.
     *
     * Outputs:
     * N/A. This is a class constructor.
     *
     * Details:
     * When instantiating an object of the MagSerialRobot class, the minimum
     * required information is the number of links in the robot. The number
     * of links is needed for sizing the dynamic arrays that describe the
     * magnetic and geometrical properties of the robot.
     *
     */
    mMagnetLocal = new Eigen::Vector3d[numLinks+1];
    mMagnetPosLocal = new Eigen::Vector3d[numLinks+1];
    mCoilMatrix = Eigen::Matrix<double,8,8>::Identity();
    for (int i = 0; i<numLinks+1; i++)
    {
        mMagnetLocal[i] << 0.0, 0.0, 0.0;
        mMagnetPosLocal[i] << 0.0, 0.0, 0.0;
    }
}

MagSerialRobot::MagSerialRobot(int numLinks, double linkLength[],
                               double linkTwist[], double linkOffset[],
                               double jointAngle[], int jointType[],
                               Eigen::Vector3d magnetLocal[],
                               Eigen::Vector3d magnetPosLocal[])
    : robF::SerialRobot(numLinks, linkLength, linkTwist, linkOffset,
                        jointAngle, jointType)
{
    /* Detailed constructor for the MagSerialRobot class.
     *
     * Inputs:
     * int numLinks                     - The number of links in the robot.
     * double linkLength[]              - An array containing the link lengths
     *                                    of the robot (often symbolized by
     *                                    "a") in units of length.
     * double linkTwist[]               - An array containing the link
     *                                    twists of the robot (often
     *                                    symbolized by "alpha") in radians.
     * double linkOffset[]              - An array containing the link
     *                                    offsets of the robot (often
     *                                    symbolized by "d") in units of
     *                                    length.
     * double jointAngle[]              - An array containing the joint
     *                                    angles of the robot (often
     *                                    symbolized by "theta") in radians.
     * int jointType[]                  - An array specifying the joint
     *                                    types of the robot (0 for
     *                                    revolute, 1 for prismatic).
     * Eigen::Vector3d magnetLocal[]    - An array of 3x1 vectors
     *                                    specifying the orientation and
     *                                    magnitude of the onboard magnets
     *                                    in each link in local (link)
     *                                    coordinates.
     * Eigen::Vector3d magnetPosLocal[] - An array of 3x1 vectors
     *                                    specifying the position of the
     *                                    onboard magnets in each link in
     *                                    local (link) coordinates.
     *
     *
     * Outputs:
     * N/A. This is a class constructor.
     *
     * Details:
     * This constructor takes the number of links and several arrays
     * containing the Denavit-Hartenberg (DH) parameters of the robot.
     * The DH parameters are stored as array members of the SerialRobot
     * object.
     * The Denavit-Hartenberg convention used here assumes that the ith
     * frame is rigidly attached to the ith link, with the z-axis collinear
     * with either the rotation axis (for a revolute joint) or the
     * translation axis (for a prismatic joint) of the joint connecting
     * links i and i+1.
     */
    mMagnetLocal = new Eigen::Vector3d[numLinks+1];
    mMagnetPosLocal = new Eigen::Vector3d[numLinks+1];
    for (int i = 0; i<numLinks+1; i++)
    {
        mMagnetLocal[i] = magnetLocal[i];
        mMagnetPosLocal[i] = magnetPosLocal[i];
    }
}

MagSerialRobot::~MagSerialRobot()
{
    /* Magnetic serial robot class destructor.
     *
     * Inputs:
     * None
     *
     * Outputs:
     * N/A. Class desctructor.
     *
     * Details:
     * The MagSerialRobot object automatically calls the destructor of its
     * parent class, so we only need to clean up the dynamic arrays for
     * the magnet dipole vectors and positions.
     */
    delete[] mMagnetLocal;
    delete[] mMagnetPosLocal;
}

Eigen::Vector3d MagSerialRobot::m_get_magnet(int linkNumber)
{
    /* This function returns the dipole moment vector of the magnet in the
     * link specified in global coordinates.
     *
     * Inputs:
     * int linkNumber - The specified link (link 0 is the base link)
     *
     * Outputs:
     * Eigen::Vector3d m - 3x1 dipole moment vector in A.m^2
     *
     */
    // Dipole vector in homogeneous coordinates
    Eigen::Vector4d m;
    Eigen::Matrix4d T = m_calc_transform_mat_global(linkNumber);
    // Augmenting the local dipole vector with 0 (converting to homogeneous
    // coordinates).
    m << mMagnetLocal[linkNumber], 0; // A.m^2
    // Applying the coordinate transform to the dipole vector in local
    // coordinates to get the dipole vector in base coordinates.
    m = T * m;
    // Return only the first 3 elements (back to non-homogeneous 3d coords)
    return m(Eigen::seq(0,2));
}

Eigen::Vector3d MagSerialRobot::m_get_magnet_pos(int linkNumber)
{
    /* This function returns the position vector of the magnet in the
     * link specified in global coordinates.
     *
     * Inputs:
     * int linkNumber - The specified link (link 0 is the base link)
     *
     * Outputs:
     * Eigen::Vector3d r - 3x1 position vector in meters
     *
     */
    // Dipole vector in homogeneous coordinates
    Eigen::Vector4d r;
    Eigen::Matrix4d T = m_calc_transform_mat_global(linkNumber);
    // Augmenting the local dipole position with 1 (converting to
    // homogeneous coordinates).
    r << mMagnetPosLocal[linkNumber], 1; // m
    // Applying the coordinate transform to the dipole vector in local
    // coordinates to get the dipole vector in base coordinates.
    r = T*r;
    // Return only the first 3 elements (back to non-homogeneous 3d coords)
    return r(Eigen::seq(0,2));
}

Eigen::Matrix<double,1,8> MagSerialRobot::m_actuation_vec(int jointNumber)
{
    /* This function returns the row vector Mv that relates the applied
     * augmented magnetic field vector beta (8x1) to the generalized
     * force Q_i at the specified joint. Q_i = M_{v,i} * beta
     *
     * Inputs:
     * int jointNumber              - The number of the joint according to
     *                                the DH convention.
     * Outputs:
     * Eigen::Matrix<double,1,8> Mv - 1x8 row vector of the actuation matrix
     *                                corresponding to the specified joint.
     */
    Eigen::Matrix<double,1,8> Mv = Eigen::Matrix<double,1,8>::Zero();
    Eigen::Matrix<double,6,1> unitTwist;
    unitTwist = m_calc_unit_twist_global(jointNumber);
    unitTwist = robF::change_screw_order(unitTwist);
    for (int i=mNumLinks; i>=jointNumber; i--)
    {
        Mv += unitTwist.transpose() * magF::calc_dipole_wrench_matrix(m_get_magnet(i), m_get_magnet_pos(i));
    }
    return Mv;
}

Eigen::Matrix<double, Eigen::Dynamic, 8> MagSerialRobot::m_calc_magnetization_matrix()
{
    /* This function returns the magnetization matrix Mb that relates the
     * applied augmented magnetic field vector beta = [b;g] (8x1) to the
     * generalized forces Q at the joints of the robot
     *      Q = Mb * beta
     * with beta specified in global coordinates.
     *
     * Inputs:
     * None
     *
     * Outputs:
     * Eigen::Matrix<double,mNumLinks,8> Mb - nx8 actuation
     *                                                       matrix.
     * Details:
     * The instantaneous rate of work dW/dt of a wrench on a twist can be
     * determined using:
     *      dW/dt = $_t' * A * $_w
     * where $_t is a twist (w;vO) in column-vector form, ' denotes the
     * transpose, A is the screw interchange matrix, and $_w is a wrench
     * (f;tO) in column-vector form. The screw interchange matrix changes
     * the order of a screw from ray-coordinate order to coordinate-ray
     * order or vice-versa. As such, it doesn't matter what order the
     * wrench and twist are in, as long as they are both the same order.
     * For a serial robot, we can find the generalized force about a given
     * joint if we use the twist of unit amplitude coordesponding to that
     * joint:
     *      Q = $_t' * A * $_w
     * where Q is the generalized force in N.m or N for revolute or
     * prismatic joints, respectively.
     * For more details, see Davidson & Hunt, Robots and Screw Theory,
     * 2004.
     * For a robot with multiple wrenches acting in parallel (i.e.
     * multiple independent forces and torques acting on the robot in
     * different locations), all wrenches acting on links distal to the
     * joint of interest can be summed. This is convenient because
     * magnetic wrenches are linear functions of the applied augmented 8x1
     * field vector beta = [b;g]. As a result:
     *      $_{w,total,i} = sum_{j=i}^{n}($_w,j)
     *      $_{w,total,i} = sum_{j=i}^{n}(Mw_j) * beta
     * where $_{w,total,i} is the total wrench acting about Joint i and
     * Mw_j is the magnetic wrench matrix corresponding to Link j. From
     * this expression, a single linear relation can be derived that
     * relates the augmented magnetic field vector to the generalized
     * forces at the robot joints:
     *      Q_i = $_{t,i}' * A * sum_{j=i}^{n}(Mw_j) * beta
     *
     *      Mb = [$_{t,1}' * A * sum_{j=1}^{n}(Mw_j)]
     *           [$_{t,2}' * A * sum_{j=2}^{n}(Mw_j)]
     *                          :
     *                          :
     *           [$_{t,n}' * A * sum_{j=n}^{n}(Mw_j)]
     *
     *      Q = Mb * beta
     */
    Eigen::MatrixXd Ma(mNumLinks,8);
    // Variable for storing the magnetic wrench matrix
    Eigen::Matrix<double,6,8> Mw = Eigen::Matrix<double,6,8>::Zero();
    // Variable for storing the unit twist of each joint
    Eigen::Matrix<double,6,1> unitTwist;
    // For each link...
    for (int i=mNumLinks; i>0; i--)
    {
        // Determine the unit twist of the preceding joint. Joint i
        // is proximal to Link i.
        unitTwist = m_calc_unit_twist_global(i);
        // Change to coordinate-ray order from ray-coordinate order.
        unitTwist = robF::change_screw_order(unitTwist);
        // Calculate the magnetic wrench matrix. The wrench on Joint i
        // is the sum of all wrenches distal to i; hence the =+ operator.
        Mw += magF::calc_dipole_wrench_matrix(m_get_magnet(i), m_get_magnet_pos(i));
        // Calculate this row of the actuation matrix.
        Ma(i-1,Eigen::all) = unitTwist.transpose() * Mw;
    }
    return Ma;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> MagSerialRobot::m_calc_applied_gen_forces(Eigen::Matrix<double, 8, 1> augField)
{
    /*
     * Calculate the generalized forces in the robot joints due to the
     * homogeneous magnetic field in the workspace generated by the
     * actuation system.
     *
     * Inputs:
     * Eigen::Matrix<double, 8, 1> augField - 8x1 homogeneous magnetic field
     *                                        vector [b;g] in T and T/m
     *
     * Outputs:
     * Eigen::Matrix<double, Eigen::Dynamic, 1> - nx1 vector of generalized
     *                                            forces in N and N.m
     *
     * Details:
     * The field actuation system will create a magnetic field in the robot
     * workspace that will apply wrenches to the magnets in the robot links.
     * With the homogeneous field assumption, we assume the augmented field
     * vector beta = [b;g] is constant throughout the workspace. Then we can
     * calculate the generalized forces with a single linear expression by
     * multiplying the augmented field vector by the robot actuation matrix.
     */
    Eigen::VectorXd Q(mNumLinks);
    Q = m_calc_magnetization_matrix()*augField;
    return Q;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> MagSerialRobot::m_calc_internal_gen_forces()
{
    /*
     * Calculate the generalized forces in the robot joints due to the
     * internal forces and torques between the magnets in the robot
     * links.
     *
     * Inputs:
     * None
     *
     * Outputs:
     * Eigen::Matrix<double, Eigen::Dynamic, 1> - nx1 vector of generalized
     *                                            forces in N and N.m
     *
     * Details:
     * The internal generalized force is the sum of wrenches from all
     * magnets proximal to a given joint on all magents distal to a
     * given joint. If i is the number of the generalized coordinate,
     * about which we wish to find the generalized force, then
     *
     * tau_{int,i} = twist_i * [DELTA] * sum_{j=i}^n (...
     *                      ...sum_{k=0}^{i-1} wrench_{k,j})
     *
     * Where twist_i is the unit twist corresponding to the ith generalized
     * coordinate, [DELTA] is the screw interchange matrix, and
     * wrench_{k,j} is the magnetic wrench caused by the field produced
     * by the magnet in link k on the magnet in link j.
     */
    Eigen::VectorXd tauInt = Eigen::VectorXd::Zero(mNumLinks);
    Eigen::Matrix<double,6,1> wrench;
    Eigen::Matrix<double,6,1> unitTwist;
    // Magnetic dipole vector for the magnet in the jth link
    Eigen::Vector3d mj;
    // Global position vector for the magnet in the jth link
    Eigen::Vector3d rj;
    // Magnetic dipole vector for the magnet in the kth link
    Eigen::Vector3d mk;
    // Global position vector for the magnet in the kth link
    Eigen::Vector3d rk;
    // Work in progress
    for (int linkNum = 1; linkNum <= mNumLinks; linkNum++)
    {
        unitTwist = robF::change_screw_order(m_calc_unit_twist_global(linkNum));
        for (int j = linkNum; j <= mNumLinks; j++)
        {
            // Get the dipole vector and position of the jth magnet.
            mj = m_get_magnet(j);
            rj = m_get_magnet_pos(j);
            for (int k = 0; k < linkNum; k++)
            {
                // Get the
                mk = m_get_magnet(k);
                rk = m_get_magnet_pos(k);
                wrench = magF::calc_wrench_between_dipoles(mk, mj, rk, rj);
                tauInt(linkNum-1) += unitTwist.transpose()*wrench;
            }
        }
    }
    return tauInt;
}

void MagSerialRobot::m_change_magnets(Eigen::Vector3d magnetLocal[],
                                Eigen::Vector3d magnetPosLocal[])
{
    /*
     * Change the position and dipole moment of magnets in the local link
     * coordinates of the robot.
     *
     * Inputs:
     * Eigen::Vector3d magnetLocal[]    - An array of 3x1 vectors
     *                                    specifying the orientation and
     *                                    magnitude of the onboard magnets
     *                                    in each link in local (link)
     *                                    coordinates.
     * Eigen::Vector3d magnetPosLocal[] - An array of 3x1 vectors
     *                                    specifying the position of the
     *                                    onboard magnets in each link in
     *                                    local (link) coordinates.
     *
     * Outputs:
     * None.
     *
     * Details:
     * This method sets the position and orientation of the magnets in the
     * local link coordinates of the robot.
     */
    for (int i = 0; i<mNumLinks+1; i++)
    {
        mMagnetLocal[i] = magnetLocal[i];
        mMagnetPosLocal[i] = magnetPosLocal[i];
    }
}

double MagSerialRobot::m_SSV(void)
{
    // Compute the singular value decomposition of the actuation matrix
    mSVD.compute(m_calc_actuation_matrix());
    // Extract the singular values and return the smallest
    mSingularValues = mSVD.singularValues();
    return mSingularValues(Eigen::last);
}

void MagSerialRobot::m_set_coil_matrix(Eigen::Matrix<double, 8, Eigen::Dynamic> coilMatrix)
{
    mCoilMatrix = coilMatrix;
}

Eigen::MatrixXd MagSerialRobot::m_calc_actuation_matrix()
{
    /* This function returns the actuation matrix Mu that relates the
     * control inputs (coil currents) u (px1) to the generalized forces
     * Q at the joints of the robot
     *      Q = Mu * u
     * with beta specified in global coordinates.
     *
     * Inputs:
     * None
     *
     * Outputs:
     * Eigen::Matrix<double,mNumLinks,8> Mu - nxp actuation matrix.
     */
    return m_calc_magnetization_matrix()*mCoilMatrix;
}

double MagSerialRobot::m_calc_min_ssv(Eigen::VectorXd qRes)
{
    /* This function calculates the singular values of the robot
     * over a grid of configurations and returns the minimum singular value
     * over the robot workspace. It then sets the robot configuration to the
     * configuration that results in the minimum singular value.
     *
     * Inputs:
     * Eigen::VectorXd qRes - nx1 vector of desired grid resolution for each
     *                        joint [rad]
     *
     * Outputs:
     * double minSSV        - The minimum smallest singular value.
     *
     * Details:
     *
     */
    double minSSV = 1e6;
    double qMax;
    double qMin;
    int numPoints;
    Eigen::VectorXd* qGridPoints = new Eigen::VectorXd[mNumLinks];
    Eigen::VectorXd qMinSSV(mNumLinks);
    // Create vectors containing the test grid values.
    for (int jointNum = 1; jointNum <= mNumLinks; jointNum++)
    {
        qMax = m_get_qMax(jointNum);
        qMin = m_get_qMin(jointNum);
        numPoints = (int)ceil((qMax-qMin)/qRes(jointNum-1));
        qGridPoints[jointNum-1] = Eigen::VectorXd::LinSpaced(numPoints, qMin, qMax);
    }
    // Begin recursive calculation of the ssv over all joint values.
    m_calc_ssv(minSSV, qMinSSV, qGridPoints);
    // Set the robot config to the config where the minimum SSV was found.
    m_set_q(qMinSSV);
    // Delete the dynamic array to avoid memory leak.
    delete[] qGridPoints;
    return minSSV;
}

double MagSerialRobot::m_calc_min_ssv()
{
    // Calculates the minimum ssv with default grid resolution
    double qResPrs = 0.1e-3; //m
    double qResRev = 1.0/180.0*M_PI; //rad
    Eigen::VectorXd qRes(mNumLinks);
    for (int jointNum = 1; jointNum <= mNumLinks; jointNum++)
    {
        if(mJointType[jointNum-1] == robF::jointPrs)
        {
            qRes(jointNum-1) = qResPrs;
        }
        else
        {
            qRes(jointNum-1) = qResRev;
        }
    }
    return m_calc_min_ssv(qRes);
}

void MagSerialRobot::m_calc_ssv(double& minSSV, Eigen::VectorXd& qMinSSV, Eigen::VectorXd qGridPoints[])
{
    m_calc_ssv(minSSV, qMinSSV, qGridPoints, 1);
}

void MagSerialRobot::m_calc_ssv(double& minSSV, Eigen::VectorXd& qMinSSV, Eigen::VectorXd qGridPoints[], int jointNum)
{
    /* This function uses recursion to calculate the smallest singular value of
     * the robot actuation matrix (tau = Mu*u) at every point on the grid
     * specified by the grid vectors in qGridPoints.
     *
     * Inputs:
     * double &minSSV - The minimum smallest singular value, passed by reference.
     * Eigen::VectorXd &qMinSSV - The configuration corresponding to the
     *                            smallest singular value, passed by reference.
     * Eigen::VectorXd *qGridPoints - An array of Eigen vector objects
     *                                specifying the grid points. For example,
     *                                qGridPoints[0] contains a vector object
     *                                that specifies the grid values for joint
     *                                1.
     * int jointNum - The number of the joint currently being iterated over.
     *                This variable is used to keep track of how far along the
     *                robot we've moved in the recursion.
     *
     * Outputs:
     * No return value. Instead, the function modifies the minSSV and qMinSSV
     * variables, which were passed by reference.
     */
    double ssv;
    // Iterate over the grid values for this joint.
    for (double q : qGridPoints[jointNum-1])
    {
        // Update the robot state for this joint.
        m_set_q(jointNum, q);
        // If we have not reached the last joint in the robot...
        if (jointNum < mNumLinks)
        {
            // ... then call self recursively to move to the next joint.
            m_calc_ssv(minSSV, qMinSSV, qGridPoints, jointNum+1);
        }
        else
        {
            // Calculate the smallest singular value for the present state.
            ssv = m_SSV();
            // If the smallest singular value is less than the previous
            // minimum smallest singular value...
            if (ssv < minSSV)
            {
                // Update the minimum ssv.
                minSSV = ssv;
                // Update the state corresponding to the minimum ssv.
                qMinSSV = m_get_q();
            }
        }
    }
}

void MagSerialRobot::m_optimize_dipole_vectors(void)
{
    // Levenberg-Marquardt optimization
    // Based on: https://medium.com/@sarvagya.vaish/levenberg-marquardt-optimization-part-2-5a71f7db27a0
    Eigen::VectorXf x(3*mNumLinks);
    int paramNum;
    float dipoleX;
    float dipoleY;
    float dipoleZ;
    float dipoleH;
    float dipoleMag;
    float azimuthAngle;
    float polarAngle;
    LMfunctor dipoleFunctor(this);
    dipoleFunctor.m = 1;
    dipoleFunctor.n = 3*mNumLinks;
    Eigen::LevenbergMarquardt<LMfunctor, float> lm(dipoleFunctor);
    // Take the existing robot parameters as the initial conditions.
    for (int linkNum = 1; linkNum <= mNumLinks; linkNum++)
    {
        dipoleX = float(mMagnetLocal[linkNum](0));
        dipoleY = float(mMagnetLocal[linkNum](1));
        dipoleZ = float(mMagnetLocal[linkNum](2));
        dipoleMag = float(mMagnetLocal[linkNum].norm());
        dipoleH = sqrt(dipoleX*dipoleX + dipoleY*dipoleY);
        azimuthAngle = float(atan2(dipoleY, dipoleX));
        polarAngle = float(atan2(dipoleZ, dipoleH));
        paramNum = 3*(linkNum-1);
        x(paramNum) = dipoleMag;
        x(paramNum+1) = azimuthAngle;
        x(paramNum+2) = polarAngle;
    }
    int status = lm.minimize(x);
    std::cout << "LM Solver status: " << status << std::endl;
    std::cout << "LM iterations: " << lm.iter << std::endl;
    std::cout << "Final parameters: " << std::endl;
    std::cout << x << std::endl << std::endl;
    // Set the robot magnets to the optimized values.
    for (int linkNum = 1; linkNum <= mNumLinks; linkNum++)
    {
        paramNum = 3*(linkNum-1);
        dipoleMag = x(paramNum);
        azimuthAngle = x(paramNum+1);
        polarAngle = x(paramNum+2);
        mMagnetLocal[linkNum] << dipoleMag*cos(azimuthAngle)*sin(polarAngle),
                                dipoleMag*sin(azimuthAngle)*sin(polarAngle),
                                dipoleMag*cos(polarAngle);
    }
}

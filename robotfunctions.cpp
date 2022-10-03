#include "robotfunctions.h"
#include <iostream>

Eigen::Matrix<double,6,1> robF::change_screw_order(Eigen::Matrix<double,6,1> screw)
{
    Eigen::Matrix<double,6,6> screwIntchngMat{
        {0,0,0,1,0,0},
        {0,0,0,0,1,0},
        {0,0,0,0,0,1},
        {1,0,0,0,0,0},
        {0,1,0,0,0,0},
        {0,0,1,0,0,0}
    };
    return screwIntchngMat * screw;
}

robF::SerialRobot::SerialRobot(int numLinks, double linkLength[], double linkTwist[], double linkOffset[], double jointAngle[], int jointType[])
{
    /* Detailed constructor for the SerialRobot class.
     *
     * Inputs:
     * int numLinks        - The number of links in the robot.
     * double linkLength[] - An array containing the link lengths of the
     *                       robot (often symbolized by "a") in units of
     *                       length.
     * double linkTwist[]  - An array containing the link twists of the
     *                       robot (often symbolized by "alpha") in
     *                       radians.
     * double linkOffset[] - An array containing the link offsets of the
     *                       robot (often symbolized by "d") in units of
     *                       length.
     * double jointAngle[] - An array containing the joint angles of the
     *                       robot (often symbolized by "theta") in
     *                       radians.
     * int jointType[]     - An array specifying the joint types of the
     *                       robot (0 for revolute, 1 for prismatic).
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
    mNumLinks = numLinks;
    mLinkLength = new double[numLinks];
    mLinkTwist = new double[numLinks];
    mLinkOffset = new double[numLinks];
    mJointAngle = new double[numLinks];
    mJointType = new int[numLinks];
    mQRange = new double[2*numLinks];
    for (int i = 0; i<numLinks; i++)
    {
        mLinkLength[i] = linkLength[i];
        mLinkTwist[i] = linkTwist[i];
        mLinkOffset[i] = linkOffset[i];
        mJointAngle[i] = jointAngle[i];
        mJointType[i] = jointType[i];
        if(mJointType[i] == jointPrs)
        {
            m_set_qRange(i, rangePrsDefault);
        }
        else
        {
            m_set_qRange(i, rangeRevDefault);
        }
    }
}

robF::SerialRobot::SerialRobot(int numLinks)
{
    /* Simplified constructor for the SerialRobot class.
     *
     * Inputs:
     * int numLinks - The number of links in the robot.
     *
     * Outputs:
     * N/A. This is a class constructor.
     *
     * Details:
     * This constructor takes the number of links and initializes all DH
     * parameters to zero.
     */
    mNumLinks = numLinks;
    mLinkLength = new double[numLinks];
    mLinkTwist = new double[numLinks];
    mLinkOffset = new double[numLinks];
    mJointAngle = new double[numLinks];
    mJointType = new int[numLinks];
    mQRange = new double[2*numLinks];
    for (int i = 0; i<numLinks; i++)
    {
        mLinkLength[i] = 0.0;
        mLinkTwist[i] = 0.0;
        mLinkOffset[i] = 0.0;
        mJointAngle[i] = 0.0;
        mJointType[i] = jointRev;
        m_set_qRange(i, rangeRevDefault);
    }
}

robF::SerialRobot::~SerialRobot()
{
    /* Destructor for the SerialRobot class.
     *
     * Inputs:
     * None
     *
     * Outputs:
     * N/A. This is a class destructor.
     *
     * Details:
     * Performs some cleanup tasks, such as deleting the dynamic arrays
     * that store the DH parameters for the robot links.
     */
    delete[] mLinkLength;
    delete[] mLinkTwist;
    delete[] mLinkOffset;
    delete[] mJointAngle;
    delete[] mJointType;
    delete[] mQRange;
}

void robF::SerialRobot::m_change_DH_params(double linkLength[], double linkTwist[], double linkOffset[], double jointAngle[], int jointType[])
{
    /*
     * This function changes the DH parameters of the robot.
     *
     * Inputs:
     * double linkLength[] - An array containing the link lengths of the
     *                       robot (often symbolized by "a") in units of
     *                       length.
     * double linkTwist[]  - An array containing the link twists of the
     *                       robot (often symbolized by "alpha") in
     *                       radians.
     * double linkOffset[] - An array containing the link offsets of the
     *                       robot (often symbolized by "d") in units of
     *                       length.
     * double jointAngle[] - An array containing the joint angles of the
     *                       robot (often symbolized by "theta") in
     *                       radians.
     * int jointType[]     - An array specifying the joint types of the
     *                       robot (0 for revolute, 1 for prismatic).
     *
     * Outputs:
     * None.
     *
     * Details:
     * This method allows the DH parameters of the robot to be changed. It
     * is useful if you want to instantiate the robot with only the number
     * of links by calling SerialRobot(int n) and then set the DH parameters
     * later. Note that the number of links is fixed, so the arrays passed to
     * this function must have a consistent size with the number of links in
     * the robot.
     */
    for (int i = 0; i<mNumLinks; i++)
    {
        mLinkLength[i] = linkLength[i];
        mLinkTwist[i] = linkTwist[i];
        mLinkOffset[i] = linkOffset[i];
        mJointAngle[i] = jointAngle[i];
        mJointType[i] = jointType[i];
    }
}

Eigen::VectorXd robF::SerialRobot::m_get_q()
{
    /* This function returns the present configuration (generalized
     * coordinates) of the robot.
     *
     * Inputs:
     * None
     *
     * Outputs:
     * Eigen::VectorXd q - A vector of the generalized coordinates in
     *                     mixed units [radians, meters].
     *
     * Details:
     * For a serial robot defined using the Denavit-Hartenberg convention,
     * the generalized coordinates are the joint angles (radians) and link
     * offsets (meters) for revolute and prismatic joints, respectively.
     */
    Eigen::VectorXd q(mNumLinks);
    for(int i = 0; i < mNumLinks; i++)
    {
        switch (mJointType[i])
        {
        case jointRev:
            // Revolute joint
            q(i) = mJointAngle[i];
            break;
        case jointPrs:
            // Prismatic joint
            q(i) = mLinkOffset[i];
            break;
        }
    }
    return q;
}

void robF::SerialRobot::m_set_q(Eigen::VectorXd q)
{
    /* This function changes the configuration (generalized coordinates)
     * of the robot.
     *
     * Inputs:
     * Eigen::VectorXd q - A vector of the generalized coordinates in
     *                     mixed units [radians, meters].
     *
     * Outputs:
     * None
     *
     * Details:
     * For a serial robot defined using the Denavit-Hartenberg convention,
     * the generalized coordinates are the joint angles (radians) and link
     * offsets (meters) for revolute and prismatic joints, respectively.
     */
    for(int i = 0; i < mNumLinks; i++)
    {
        switch (mJointType[i])
        {
        case jointRev:
            // Revolute joint
            mJointAngle[i] = q(i);
            break;
        case jointPrs:
            // Prismatic joint
            mLinkOffset[i] = q(i);
            break;
        }
    }
}

void robF::SerialRobot::m_set_q(int jointNumber, double q)
{
    /* This method changes the value of the generalized coordinate of the
     * specified joint.
     *
     * Inputs:
     * int jointNumber - The number of the joint according to the DH
     *                   convention (1 to n).
     * double q        - The desired value of the generalized coordinate
     *                   corresponding to the specified joint.
     *
     * Outputs:
     * None
     *
     * Details:
     * Note: In the DH convention that I use, the ith joint precedes the
     * ith link. For example, the first link is the base link (Link 0),
     * which is connected to Link 1 by Joint 1. Link 1 then connects to
     * Link 2 by Joint 2.
     */
    switch (mJointType[jointNumber-1])
    {
    case jointRev:
        // Revolute joint
        mJointAngle[jointNumber-1] = q;
        break;
    case jointPrs:
        // Prismatic joint
        mLinkOffset[jointNumber-1] = q;
        break;
    }
}

void robF::SerialRobot::m_set_Tbase(Eigen::Matrix4d T)
{
    /* This function modifies the Tbase member of the serial robot (the
     * affine transformation matrix from the base frame to the global
     * frame).
     *
     * Inputs:
     * Eigen::Matrix4d T - 4x4 affine transformation matrix from the robot
     *                     base frame to the global frame.
     *
     * Outputs:
     * None
     */
    mTBase = T;
}

Eigen::Matrix4d robF::SerialRobot::m_calc_transform_mat_single_link(int fromFrame)
{
    /* This function calculates the 4x4 homogeneous transformation matrix
     * from frame i to frame i-1.
     *
     * Inputs:
     * double linkLength - The length of link i (often symbolized by "a")
     *                     in units of length.
     * double linkTwist  - The twist of link i (often symbolized by
     *                     "alpha") in radians.
     * double linkOffset - The offset of the link i (often symbolized by
     *                     "d") in units of length.
     * double jointAnlge - The joint angle of link i (often symbolized by
     *                     "theta") in radians.
     * Outputs:
     * Eigen::Matrix4d T - A 4x4 homogeneous transformation matrix
     *                     (affine transformation of 3D points and
     *                     vectors) in mixed units.
     *
     * Details:
     * The transformation matrix is often written as T_i^j, where i is the
     * starting frame (the frame in which a point or vector is given) and
     * j is the ending frame (the frame in which we would like to express
     * the given point or vector). The most common transformation is from
     * a local link frame i to the base frame 0: T_i^0.
     * The Denavit-Hartenberg convention used here assumes that the ith
     * frame is rigidly attached to the ith link, with the z-axis collinear
     * with either the rotation axis (for a revolute joint) or the
     * translation axis (for a prismatic joint) of the joint connecting
     * links i and i+1.
     * The transformation matrix from link i to the preceding link i-1 can
     * be calculated from the Denavit-Hartenberg parameters of the ith
     * link. This transformation matrix is often written as A_i:
     *      T_i^{i-1} = A_i
     */
    Eigen::Matrix4d T;
    double theta = mJointAngle[fromFrame]; //rad
    double alpha = mLinkTwist[fromFrame]; //rad
    double a = mLinkLength[fromFrame]; //meter
    double d = mLinkOffset[fromFrame]; //meter
    T << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta),
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
                0.0,             sin(alpha),             cos(alpha),            d,
                0.0,                    0.0,                    0.0,          1.0;
    return T;
}

Eigen::Matrix4d robF::SerialRobot::m_calc_transform_mat(int fromFrame, int toFrame)
{
    /* This method returns the transformation matrix between two link
     * frames for the serial robot in its present state.
     *
     * Inputs:
     * int fromFrame       - The index of the frame in which coordinates
     *                       are presently expressed (local link frame).
     * int toFrame         - The index of the frame in which you want the
     *                       coordinates to be expressed. E.g. 0 is the
     *                       base frame.
     *
     * Outputs:
     * Eigen::Matrix4d T   - A 4x4 homogeneous transformation matrix
     *                       (affine transformation of 3D points and
     *                       vectors) in mixed units.
     *
     * Details:
     * The transformation matrix is often written as T_i^j, where i is the
     * starting frame (the frame in which a point or vector is given) and
     * j is the ending frame (the frame in which we would like to express
     * the given point or vector). The most common transformation is from
     * a local link frame i to the base frame 0: T_i^0.
     * The Denavit-Hartenberg convention used here assumes that the ith
     * frame is rigidly attached to the ith link, with the z-axis collinear
     * with either the rotation axis (for a revolute joint) or the
     * translation axis (for a prismatic joint) of the joint connecting
     * links i and i+1.
     * The transformation matrix from link i to the preceding link i-1 can
     * be calculated from the Denavit-Hartenberg parameters of the ith
     * link. This transformation matrix is often written as A_i:
     *      T_i^{i-1} = A_i
     * The transformation between two non-adjacent links (J<i) can then be
     * determined from the product of the A_i matrices:
     *      T_i^j = A_{j+1} * A_{j+2} ... * A_{i-1} * A_i
     * The transformation matrix has a rotation component R_i^j (the 3x3
     * rotation matrix) and a translation component o_i^j (the 3x1
     * translation vector):
     *      T_i^j = [R_i^j, o_i^j]
     *              [0,0,0,     1]
     * Conveniently, the inverse transformation matrix can be calculated
     * simply using the following formula:
     *      T_j^i = (T_i^j)^-1 = [(R_i^j)', -(R_i^j)'*o_i^j]
     *                           [   0,0,0,               1]
     * where ' denotes the matrix transpose (similar to MATLAB notation).
     */
    // The 4x4 homogeneous transformation matrix
    Eigen::Matrix4d T;
    // A 3x3 rotation matrix (used in finding the inverse of T)
    Eigen::Matrix3d R;
    // A 3x1 displacement vector (used in finding the inverse of T)
    Eigen::Vector3d o;
    // Assign the identity matrix to the transformation matrix (equivalent
    // to a null transformation). This is returned without modification if
    // fromFrame == toFrame.
    T = Eigen::Matrix4d::Identity();
    // Check whether both of the requested frames are within the range from
    // 0 to numLinks
    if ((0 <= fromFrame) && (fromFrame <= mNumLinks) && (0 <= toFrame) && (toFrame <= mNumLinks))
    {
        // Check whether the starting frame is distal or proximal to the
        // ending frame.
        if (fromFrame - toFrame > 0)
        {
            // The starting frame is distal to the ending frame.
            for (int i = toFrame; i < fromFrame; i++)
            {
                T *= m_calc_transform_mat_single_link(i);
            }
        }
        else if (fromFrame - toFrame < 0)
        {
            // The starting frame is proximal to the ending frame.
            // First, find the transformation from the ending frame to the
            // starting frame T_j^i (i.e. from the distal frame to the proximal
            // frame).
            for (int i = fromFrame; i < toFrame; i++)
            {
                T *= SerialRobot::m_calc_transform_mat_single_link(i);
            }
            // Extract the rotation matrix and translation vector (for
            // readability purposes).
            R = T(Eigen::seq(0,2),Eigen::seq(0,2));
            o = T(Eigen::seq(0,2),Eigen::last);
            // Then, invert T_j^i to get T_i^j.
            T << R.transpose(), -R.transpose()*o,
                       0, 0, 0,                1;
        }
    }
    else
    {
        std::cout << "Requested frame exceeds number of links." << std::endl;
    }
    return T;
}

Eigen::Matrix4d robF::SerialRobot::m_calc_transform_mat(int fromFrame)
{
    /* This method returns the transformation matrix from the specified
     * frame to the base frame of the serial robot.
     *
     * Inputs:
     * int fromFrame       - The index of the frame in which coordinates
     *                       are presently expressed (local link frame).
     *
     * Outputs:
     * Eigen::Matrix4d T   - A 4x4 homogeneous transformation matrix
     *                       (affine transformation of 3D points and
     *                       vectors) in mixed units.
     */
    // The 4x4 homogeneous transformation matrix
    Eigen::Matrix4d T;
    T = m_calc_transform_mat(fromFrame, 0);
    return T;
}

Eigen::Matrix4d robF::SerialRobot::m_calc_transform_mat()
{
    /* This method returns the transformation matrix from the end-effector
     * frame to the base frame of the serial robot.
     *
     * Inputs:
     * None
     *
     * Outputs:
     * Eigen::Matrix4d T   - A 4x4 homogeneous transformation matrix
     *                       (affine transformation of 3D points and
     *                       vectors) in mixed units.
     */
    Eigen::Matrix4d T;
    T = m_calc_transform_mat(mNumLinks);
    return T;
}

Eigen::Matrix4d robF::SerialRobot::m_calc_transform_mat_global(int fromFrame)
{
    /* This method returns the transformation matrix from the specified
     * frame of the serial robot to the global frame.
     *
     * Inputs:
     * int fromFrame       - The index of the frame in which coordinates
     *                       are presently expressed (local link frame).
     *
     * Outputs:
     * Eigen::Matrix4d T   - A 4x4 homogeneous transformation matrix
     *                       (affine transformation of 3D points and
     *                       vectors) in mixed units.
     */
    Eigen::Matrix4d T;
    T = mTBase * m_calc_transform_mat(fromFrame);
    return T;
}

Eigen::Matrix<double,6,1> robF::SerialRobot::m_calc_unit_twist(int jointNumber)
{
    /* This method calculates the twist of unit amplitude in base
     * coordinates corresponding to the joint specified by jointNumber in
     * the serial robot.
     *
     * Inputs:
     * int jointNumber - The number of the joint according to the DH
     *                   convention.
     *
     * Outputs:
     * Eigen::Matrix<double,6,1> jointTwist - The twist of unit magnitude
     *                                        corresponding to the joint.
     *
     * Details:
     * A twist is a screw $ = (w; vO) that describes the instantaneous
     * kinematics of a rigid body. I use the ray-coordinate convention to
     * describe screws. The 3x1 vector w is the angular velocity of the
     * rotating body. The 3x1 vector vO is the linear velocity of an
     * imaginary point on the body located at the origin of the present
     * frame:
     *      vO = v + r x w
     * where v is the linear velocity of the body at a point along the
     * rotational axis and r is a vector from the origin of the frame to
     * the rotational axis.
     * For a revolute joint with a reference frame defined with the DH
     * convention, the twist of unit magnitude corresponding to the ith
     * joint is
     *      $ = (0_z_(i-1); 0_o_(i-1) x 0_z_(i-1))
     * where 0_z_(i-1) is the z-axis unit vector of the i-1 frame
     * expressed in base frame (0 frame) coordinates, 0_o_(i-1) is the
     * position vector of the i-1 frame expressed relative to the base
     * frame, and "x" denotes the vector cross product.
     * Similarly, for a prismatic joint with a reference frame defined
     * with the DH convention, the twist of unit magnitude corresponding
     * to the ith joint is
     *      $ = (0, 0, 0; 0_z_(i-1))
     * For more details, see Davidson & Hunt, Robots and Screw Theory,
     * 2004.
     * Note: In the DH convention that I use, the ith joint precedes the
     * ith link. For example, the first link is the base link (Link 0),
     * which is connected to Link 1 by Joint 1. Link 1 then connects to
     * Link 2 by Joint 2.
     */
    Eigen::Matrix<double,6,1> jointTwist;
    // The number of the reference frame corresponding to the given joint
    int frameNumber = jointNumber - 1;
    // The transformation matrix from the frame corresponding to the joint
    // specified by jointNumber. If i = jointNumber, then the i-1 frame
    // has a z-axis that is collinear with the motion axis of joint i.
    Eigen::Matrix4d TMat = m_calc_transform_mat(frameNumber);
    // Initialize the velocity and position of the joint with zeros.
    Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    // Check the joint type
    switch (mJointType[frameNumber])
    {
        case jointRev:
            // The joint is revolute
            angularVelocity = TMat(Eigen::seq(0,2),2);
            position = TMat(Eigen::seq(0,2),3);
            break;
        case jointPrs:
            // The joint is prismatic
            linearVelocity = TMat(Eigen::seq(0,2),2);
            break;
    }
    // Calculate the joint twist.
    jointTwist << angularVelocity,
                  linearVelocity + position.cross(angularVelocity);
    return jointTwist;
}

Eigen::Matrix<double,6,1> robF::SerialRobot::m_calc_unit_twist_global(int jointNumber)
{
    /* This method calculates the twist of unit amplitude in global
     * coordinates corresponding to the joint specified by jointNumber in
     * the serial robot.
     *
     * Inputs:
     * int jointNumber - The number of the joint according to the DH
     *                   convention.
     *
     * Outputs:
     * Eigen::Matrix<double,6,1> jointTwist - The twist of unit magnitude
     *                                        corresponding to the joint.
     *
     * Details:
     * A twist is a screw $ = (w; vO) that describes the instantaneous
     * kinematics of a rigid body. I use the ray-coordinate convention to
     * describe screws. The 3x1 vector w is the angular velocity of the
     * rotating body. The 3x1 vector vO is the linear velocity of an
     * imaginary point on the body located at the origin of the present
     * frame:
     *      vO = v + r x w
     * where v is the linear velocity of the body at a point along the
     * rotational axis and r is a vector from the origin of the frame to
     * the rotational axis.
     * For a revolute joint with a reference frame defined with the DH
     * convention, the twist of unit magnitude corresponding to the ith
     * joint is
     *      $ = (0_z_(i-1); 0_o_(i-1) x 0_z_(i-1))
     * where 0_z_(i-1) is the z-axis unit vector of the i-1 frame
     * expressed in base frame (0 frame) coordinates, 0_o_(i-1) is the
     * position vector of the i-1 frame expressed relative to the base
     * frame, and "x" denotes the vector cross product.
     * Similarly, for a prismatic joint with a reference frame defined
     * with the DH convention, the twist of unit magnitude corresponding
     * to the ith joint is
     *      $ = (0, 0, 0; 0_z_(i-1))
     * For more details, see Davidson & Hunt, Robots and Screw Theory,
     * 2004.
     * Note: In the DH convention that I use, the ith joint precedes the
     * ith link. For example, the first link is the base link (Link 0),
     * which is connected to Link 1 by Joint 1. Link 1 then connects to
     * Link 2 by Joint 2.
     */
    Eigen::Matrix<double,6,1> jointTwist;
    // The number of the reference frame corresponding to the given joint
    int frameNumber = jointNumber - 1;
    // The transformation matrix from the frame corresponding to the joint
    // specified by jointNumber. If i = jointNumber, then the i-1 frame
    // has a z-axis that is collinear with the motion axis of joint i.
    Eigen::Matrix4d TMat = m_calc_transform_mat_global(frameNumber);
    // Initialize the velocity and position of the joint with zeros.
    Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    // Check the joint type
    switch (mJointType[frameNumber])
    {
        case jointRev:
            // The joint is revolute
            angularVelocity = TMat(Eigen::seq(0,2),2);
            position = TMat(Eigen::seq(0,2),3);
            break;
        case jointPrs:
            // The joint is prismatic
            linearVelocity = TMat(Eigen::seq(0,2),2);
            break;
    }
    // Calculate the joint twist.
    jointTwist << angularVelocity,
                  linearVelocity + position.cross(angularVelocity);
    return jointTwist;
}

double robF::SerialRobot::m_calc_gen_force_from_wrench(int jointNumber, Eigen::Matrix<double,6,1> wrench)
{
    /* This function calculates the generalized force about the specified
     * joint of the robot due to a distally-applied wrench.
     *
     * Inputs:
     * int jointNumber                   - The number of the joint according to
     *                                     the DH convention.
     * Eigen::Matrix<double,6,1> wrench  - The applied wrench in global coords.
     *
     * Outputs:
     * double Q                  - The generalized force about the joint.
     *
     * Details:
     * The instantaneous rate of work dW/dt of a wrench on a twist can be
     * determined using:
     *      dW/dt = $_t'*[A]*$_w
     * where $_t is a twist (w;vO) in column-vector form, ' denotes the
     * transpose, [A] is the screw interchange matrix, and $_w is a wrench
     * (f;tO) in column-vector form. The screw interchange matrix changes
     * the order of a screw from ray-coordinate order to coordinate-ray
     * order or vice-versa. As such, it doesn't matter what order the
     * wrench and twist are in, as long as they are both the same order.
     * For a serial robot, we can find the generalized force about a given
     * joint if we use the twist of unit amplitude coordesponding to that
     * joint:
     *      Q = $_t'*[A]*$_w
     * where Q is the generalized force in N.m or N for revolute or
     * prismatic joints, respectively.
     * For more details, see Davidson & Hunt, Robots and Screw Theory,
     * 2004.
     */
    double Q = 0.0;
    Eigen::Matrix<double,6,1> twist = m_calc_unit_twist_global(jointNumber);
    Q = twist.transpose() * robF::change_screw_order(wrench);
    return Q;
}

void robF::SerialRobot::m_set_qRange(Eigen::MatrixXd qRange)
{
    /* This function sets the allowable range of the joint values.
     *
     * Inputs:
     * Eigen::MatrixXd qRange - The nx2 matrix of ranges [min,max]
     */
    for(int i=0; i < mNumLinks; i++)
    {
        // Set lower limit for this joint value
        mQRange[2*i] = qRange(i,0); //rad
        // Set upper limit for this joint value
        mQRange[2*i+1] = qRange(i,1); //rad
    }
}

void robF::SerialRobot::m_set_qRange(int jointNumber, Eigen::Vector2d qRange)
{
    /* This function sets the allowable range of the joint values.
     *
     * Inputs:
     * int jointNumber - The jointNumber whose range is to be set.
     * Eigen::MatrixXd qRange - The 2x1 vector of ranges [min,max]
     */
    // Set lower limit for this joint value
    mQRange[2*jointNumber] = qRange(0); //rad or m
    // Set upper limit for this joint value
    mQRange[2*jointNumber+1] = qRange(1); //rad or m
}

void robF::SerialRobot::m_display_properties()
{
    std::cout << "-------------------------------------------------------------------------------" << std::endl;
    std::cout << "*************************** Serial Robot Properties ***************************" << std::endl;
    std::cout << "DH Parameters:" << std::endl;
    for (int linkNum = 0; linkNum < mNumLinks; linkNum++)
    {
        std::cout << "Link " << linkNum+1 << std::endl;
        std::cout << "a     = " << mLinkLength[linkNum] << " m" << std::endl;
        std::cout << "alpha = " << mLinkTwist[linkNum] << " rad" << std::endl;
        std::cout << "d     = " << mLinkOffset[linkNum] << " m";
        if(mJointType[linkNum] == jointPrs)
        {
            std::cout << " <- q";
        }
        std::cout << std::endl;
        std::cout << "theta = " << mJointAngle[linkNum] << " rad";
        if(mJointType[linkNum] == jointRev)
        {
            std::cout << " <- q";
        }
        std::cout << std::endl;
        std::cout << mQRange[2*linkNum] << " < q < " << mQRange[2*linkNum+1] << std::endl;
        std::cout << "****************************************" << std::endl;
    }
    std::cout << "-------------------------------------------------------------------------------" << std::endl;
}

double robF::SerialRobot::m_get_qMax(int jointNumber)
{
    return mQRange[2*(jointNumber-1)+1];
}

double robF::SerialRobot::m_get_qMin(int jointNumber)
{
    return mQRange[2*(jointNumber-1)];
}

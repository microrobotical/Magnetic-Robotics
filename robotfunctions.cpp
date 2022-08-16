#include "robotfunctions.h"
#include <iostream>

Eigen::Matrix<double,6,1> robF::changeScrewOrder(Eigen::Matrix<double,6,1> screw)
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

robF::serialRobot::serialRobot(int numLinks, double linkLength[], double linkTwist[], double linkOffset[], double jointAngle[], int jointType[])
{
    /* Detailed constructor for the serialRobot class.
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
     * The DH parameters are stored as array members of the serialRobot
     * object.
     * The Denavit-Hartenberg convention used here assumes that the ith
     * frame is rigidly attached to the ith link, with the z-axis collinear
     * with either the rotation axis (for a revolute joint) or the
     * translation axis (for a prismatic joint) of the joint connecting
     * links i and i+1.
     */
    serialRobot::numLinks = numLinks;
    serialRobot::linkLength = new double[numLinks];
    serialRobot::linkTwist = new double[numLinks];
    serialRobot::linkOffset = new double[numLinks];
    serialRobot::jointAngle = new double[numLinks];
    serialRobot::jointType = new int[numLinks];
    for (int i = 0; i<numLinks; i++)
    {
        serialRobot::linkLength[i] = linkLength[i];
        serialRobot::linkTwist[i] = linkTwist[i];
        serialRobot::linkOffset[i] = linkOffset[i];
        serialRobot::jointAngle[i] = jointAngle[i];
        serialRobot::jointType[i] = jointType[i];
    }
}

robF::serialRobot::serialRobot(int numLinks)
{
    /* Simplified constructor for the serialRobot class.
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
    serialRobot::numLinks = numLinks;
    serialRobot::linkLength = new double[numLinks];
    serialRobot::linkTwist = new double[numLinks];
    serialRobot::linkOffset = new double[numLinks];
    serialRobot::jointAngle = new double[numLinks];
    serialRobot::jointType = new int[numLinks];
    for (int i = 0; i<numLinks; i++)
    {
        serialRobot::linkLength[i] = 0.0;
        serialRobot::linkTwist[i] = 0.0;
        serialRobot::linkOffset[i] = 0.0;
        serialRobot::jointAngle[i] = 0.0;
        serialRobot::jointType[i] = 0;
    }
}

robF::serialRobot::~serialRobot()
{
    /* Destructor for the serialRobot class.
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
    delete[] serialRobot::linkLength;
    delete[] serialRobot::linkTwist;
    delete[] serialRobot::linkOffset;
    delete[] serialRobot::jointAngle;
    delete[] serialRobot::jointType;
}

Eigen::VectorXd robF::serialRobot::getq()
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
    Eigen::VectorXd q(serialRobot::numLinks);
    for(int i = 0; i < serialRobot::numLinks; i++)
    {
        switch (serialRobot::jointType[i])
        {
        case JOINTREV:
            // Revolute joint
            q(i) = serialRobot::jointAngle[i];
            break;
        case JOINTPRS:
            // Prismatic joint
            q(i) = serialRobot::linkOffset[i];
            break;
        }
    }
    return q;
}

void robF::serialRobot::setq(Eigen::VectorXd q)
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
    for(int i = 0; i < serialRobot::numLinks; i++)
    {
        switch (serialRobot::jointType[i])
        {
        case JOINTREV:
            // Revolute joint
            serialRobot::jointAngle[i] = q(i);
            break;
        case JOINTPRS:
            // Prismatic joint
            serialRobot::linkOffset[i] = q(i);
            break;
        }
    }
}

void robF::serialRobot::setq(int jointNumber, double q)
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
    switch (serialRobot::jointType[jointNumber-1])
    {
    case JOINTREV:
        // Revolute joint
        serialRobot::jointAngle[jointNumber-1] = q;
        break;
    case JOINTPRS:
        // Prismatic joint
        serialRobot::linkOffset[jointNumber-1] = q;
        break;
    }
}

void robF::serialRobot::setTbase(Eigen::Matrix4d T)
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
    serialRobot::Tbase = T;
}

Eigen::Matrix4d robF::serialRobot::transformMatSingleLink(int fromFrame)
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
    double theta = serialRobot::jointAngle[fromFrame]; //rad
    double alpha = serialRobot::linkTwist[fromFrame]; //rad
    double a = serialRobot::linkLength[fromFrame]; //meter
    double d = serialRobot::linkOffset[fromFrame]; //meter
    T << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta),
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
                0.0,             sin(alpha),             cos(alpha),            d,
                0.0,                    0.0,                    0.0,          1.0;
    return T;
}

Eigen::Matrix4d robF::serialRobot::transformMat(int fromFrame, int toFrame)
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
    if ((0 <= fromFrame) && (fromFrame <= serialRobot::numLinks) && (0 <= toFrame) && (toFrame <= serialRobot::numLinks))
    {
        // Check whether the starting frame is distal or proximal to the
        // ending frame.
        if (fromFrame - toFrame > 0)
        {
            // The starting frame is distal to the ending frame.
            for (int i = toFrame; i < fromFrame; i++)
            {
                T *= serialRobot::transformMatSingleLink(i);
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
                T *= serialRobot::transformMatSingleLink(i);
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

Eigen::Matrix4d robF::serialRobot::transformMat(int fromFrame)
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
    T = serialRobot::transformMat(fromFrame, 0);
    return T;
}

Eigen::Matrix4d robF::serialRobot::transformMat()
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
    T = serialRobot::transformMat(serialRobot::numLinks);
    return T;
}

Eigen::Matrix4d robF::serialRobot::transformMatGlobal(int fromFrame)
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
    T = serialRobot::Tbase * serialRobot::transformMat(fromFrame);
    return T;
}

Eigen::Matrix<double,6,1> robF::serialRobot::unitTwist(int jointNumber)
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
    Eigen::Matrix4d TMat = serialRobot::transformMat(frameNumber);
    // Initialize the velocity and position of the joint with zeros.
    Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    // Check the joint type
    switch (serialRobot::jointType[frameNumber])
    {
        case JOINTREV:
            // The joint is revolute
            angularVelocity = TMat(Eigen::seq(0,2),2);
            position = TMat(Eigen::seq(0,2),3);
            break;
        case JOINTPRS:
            // The joint is prismatic
            linearVelocity = TMat(Eigen::seq(0,2),2);
            break;
    }
    // Calculate the joint twist.
    jointTwist << angularVelocity,
                  linearVelocity + position.cross(angularVelocity);
    return jointTwist;
}

Eigen::Matrix<double,6,1> robF::serialRobot::unitTwistGlobal(int jointNumber)
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
    Eigen::Matrix4d TMat = serialRobot::transformMatGlobal(frameNumber);
    // Initialize the velocity and position of the joint with zeros.
    Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    // Check the joint type
    switch (serialRobot::jointType[frameNumber])
    {
        case JOINTREV:
            // The joint is revolute
            angularVelocity = TMat(Eigen::seq(0,2),2);
            position = TMat(Eigen::seq(0,2),3);
            break;
        case JOINTPRS:
            // The joint is prismatic
            linearVelocity = TMat(Eigen::seq(0,2),2);
            break;
    }
    // Calculate the joint twist.
    jointTwist << angularVelocity,
                  linearVelocity + position.cross(angularVelocity);
    return jointTwist;
}

double robF::serialRobot::applyWrenchToJoint(int jointNumber, Eigen::Matrix<double,6,1> wrench)
{
    /* This function calculates the generalized force about the specified
     * joint of the robot due to a distally-applied wrench.
     *
     * Inputs:
     * int jointNumber           - The number of the joint according to
     *                             the DH convention.
     * Eigen::Matrix<double,6,1> - The applied wrench.
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
    Eigen::Matrix<double,6,1> twist = serialRobot::unitTwist(jointNumber);
    Q = twist.transpose() * robF::changeScrewOrder(wrench);
    return Q;
}


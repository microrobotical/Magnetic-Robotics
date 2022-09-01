#include "magserialrobot.h"

magSerialRobot::magSerialRobot(int numLinks) : robF::SerialRobot(numLinks)
{
    /* Simplified constructor for the magSerialRobot class.
     *
     * Inputs:
     * int numLinks - The number of links in the robot.
     *
     * Outputs:
     * N/A. This is a class constructor.
     *
     * Details:
     * When instantiating an object of the magSerialRobot class, the minimum
     * required information is the number of links in the robot. The number
     * of links is needed for sizing the dynamic arrays that describe the
     * magnetic and geometrical properties of the robot.
     *
     */
    mMagnetLocal = new Eigen::Vector3d[numLinks+1];
    mMagnetPosLocal = new Eigen::Vector3d[numLinks+1];
    for (int i = 0; i<numLinks+1; i++)
    {
        mMagnetLocal[i] << 0.0, 0.0, 0.0;
        mMagnetPosLocal[i] << 0.0, 0.0, 0.0;
    }
}

magSerialRobot::magSerialRobot(int numLinks, double linkLength[],
                               double linkTwist[], double linkOffset[],
                               double jointAngle[], int jointType[],
                               Eigen::Vector3d magnetLocal[],
                               Eigen::Vector3d magnetPosLocal[])
    : robF::SerialRobot(numLinks, linkLength, linkTwist, linkOffset,
                        jointAngle, jointType)
{
    /* Detailed constructor for the magSerialRobot class.
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

magSerialRobot::~magSerialRobot()
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
     * The magSerialRobot object automatically calls the destructor of its
     * parent class, so we only need to clean up the dynamic arrays for
     * the magnet dipole vectors and positions.
     */
    delete[] mMagnetLocal;
    delete[] mMagnetPosLocal;
}

Eigen::Vector3d magSerialRobot::m_get_magnet(int linkNumber)
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

Eigen::Vector3d magSerialRobot::m_get_magnet_pos(int linkNumber)
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

Eigen::Matrix<double,1,8> magSerialRobot::m_actuation_vec(int jointNumber)
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
    unitTwist = robF::changeScrewOrder(unitTwist);
    for (int i=mNumLinks; i>=jointNumber; i--)
    {
        Mv += unitTwist.transpose() * magF::dipoleWrenchMatrix(m_get_magnet(i), m_get_magnet_pos(i));
    }
    return Mv;
}

Eigen::Matrix<double, Eigen::Dynamic, 8> magSerialRobot::m_get_actuation_matrix()
{
    /* This function returns the actuation matrix Ma that relates the
     * applied augmented magnetic field vector beta = [b;g] (8x1) to the
     * generalized forces Q at the joints of the robot
     *      Q = Ma * beta
     * with beta specified in global coordinates.
     *
     * Inputs:
     * None
     *
     * Outputs:
     * Eigen::Matrix<double,mNumLinks,8> Ma - nx8 actuation
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
     *      Ma = [$_{t,1}' * A * sum_{j=1}^{n}(Mw_j)]
     *           [$_{t,2}' * A * sum_{j=2}^{n}(Mw_j)]
     *                          :
     *                          :
     *           [$_{t,n}' * A * sum_{j=n}^{n}(Mw_j)]
     *
     *      Q = Ma * beta
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
        unitTwist = robF::changeScrewOrder(unitTwist);
        // Calculate the magnetic wrench matrix. The wrench on Joint i
        // is the sum of all wrenches distal to i; hence the =+ operator.
        Mw += magF::dipoleWrenchMatrix(m_get_magnet(i), m_get_magnet_pos(i));
        // Calculate this row of the actuation matrix.
        Ma(i-1,Eigen::all) = unitTwist.transpose() * Mw;
    }
    return Ma;
}

void magSerialRobot::m_set_magnets(Eigen::Vector3d magnetLocal[],
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

#include "magfunctions.h"

Eigen::Vector3d magF::calc_dipole_field(Eigen::Vector3d dipoleMoment, Eigen::Vector3d relPosition)
{
    /* This function returns the magnetic field vector at a point in space
     * defined by a relative position vector and the dipole moment
     * vector.
     *
     * Inputs:
     * Eigen::Vector3d dipoleMoment - 3x1 point dipole moment vector in A.m^2
     * Eigen::Vector3d position     - 3x1 position vector in m
     *
     * Outputs:
     * Eigen::Vector3d field        - 3x1 magnetic field vector in tesla
     *
     * Details:
     * See p. 60 Abbott et al., "Magnetic Methods in Robotics," Ann. Rev.
     * Ctrl. Robot. Auton. Syst., vol. 3, pp. 57-90, 2020.
     */
    Eigen::Vector3d field = Eigen::Vector3d::Zero(); // tesla
    double distance = relPosition.norm(); //meter
    field = magF::mu0/(4*M_PI*pow(distance,5.0))*(3*relPosition*relPosition.transpose() - pow(distance,2.0)*Eigen::Matrix3d::Identity())*dipoleMoment; // tesla
    return field;
}

Eigen::Matrix<double,6,1> magF::calc_wrench_on_dipole(Eigen::Vector3d dipoleMoment, Eigen::Vector3d position, Eigen::Matrix<double,8,1> augField)
{
    /* This function returns the wrench on a magnetic dipole caused by the
     * surrounding magnetic field and gradient at the dipole position.
     *
     * Inputs:
     * Eigen::Vector3d dipoleMoment       - 3x1 point dipole moment vector in
     *                                      A.m^2
     * Eigen::Vector3d position           - 3x1 position vector in m
     * Eigen::Matrix<double,8,1> augField - 8x1 augmented field vector [b;g]
     *                                      in T and T/m
     *
     * Outputs:
     * Eigen::Matrix<double,6,1> wrench   - 6x1 wrench in ray-coord order
     *                                      (f;tO) in N and N.m
     *
     * Details:
     * See p. 60 Abbott et al., "Magnetic Methods in Robotics," Ann. Rev.
     * Ctrl. Robot. Auton. Syst., vol. 3, pp. 57-90, 2020.
     */
    Eigen::Matrix<double,6,1> wrench;
    wrench = calc_dipole_wrench_matrix(dipoleMoment, position) * augField;
    return wrench;
}

Eigen::Matrix3d magF::calc_dipole_torque_matrix(Eigen::Vector3d dipoleMoment)
{
    /*
     * This function returns the matrix Mt that defines the torque
     * experienced by a dipole moment in a magnetic field.
     *
     * Inputs:
     * Eigen::Vector3d dipoleMoment - 3x1 point dipole moment vector in A.m^2
     *
     * Outputs:
     * Eigen::Matrix3d Mt - 3x3 dipole torque matrix in N.m/T
     *
     * Details:
     * See p. 68 Abbott et al., "Magnetic Methods in Robotics," Ann. Rev.
     * Ctrl. Robot. Auton. Syst., vol. 3, pp. 57-90, 2020.
     */
    Eigen::Matrix3d Mt;
    Mt = calc_xprod_skew_mat(dipoleMoment);
    return Mt;
}

Eigen::Matrix<double,3,5> magF::calc_dipole_force_matrix(Eigen::Vector3d dipoleMoment)
{
    /* This function returns the matrix Mf that defines the force
     * experienced by a dipole moment in a magnetic gradient.
     *
     * Inputs:
     * Eigen::Vector3d dipoleMoment - 3x1 point dipole moment vector in A.m^2
     *
     * Outputs:
     * Eigen::Matrix3d Mf - 3x5 dipole torque matrix in N.m/T
     *
     * Details:
     * At a point in space the gradient of the magnetic field can be fully
     * defined by only five gradients. By convention, the following
     * gradients are often used:
     *      g = [g_xx
     *           g_xy
     *           g_xz
     *           g_yy
     *           g_yz]
     * Where g_uv = dbu/dv (d being the partial derivative operator).
     * The relationship between the force on a magnetic dipole and the
     * magnetic field gradient is linear:
     *      f = Mf g
     * This function returns Mf given the dipole moment vector.
     * See p. 68 Abbott et al., "Magnetic Methods in Robotics," Ann. Rev.
     * Ctrl. Robot. Auton. Syst., vol. 3, pp. 57-90, 2020.
     */
    Eigen::Matrix<double,3,5> Mf;
    Mf <<  dipoleMoment(0), dipoleMoment(1), dipoleMoment(2),                0,               0,
                         0, dipoleMoment(0),               0,  dipoleMoment(1), dipoleMoment(2),
          -dipoleMoment(2),               0, dipoleMoment(0), -dipoleMoment(2), dipoleMoment(1);
    return Mf;
}

Eigen::Matrix<double,6,8> magF::calc_dipole_wrench_matrix(Eigen::Vector3d dipoleMoment, Eigen::Vector3d position)
{
    /* This function returns the matrix Mw that defines the wrench
     * experienced by a dipole moment in a magnetic field and gradient.
     *
     * Inputs:
     * Eigen::Vector3d dipoleMoment - 3x1 point dipole moment vector in A.m^2
     * Eigen::Vector3d position     - 3x1 position vector of the point
     *                                dipole in m.
     *
     * Outputs:
     * Eigen::Matrix<double,6,8> Mw - 6x8 dipole wrench matrix in mixed
     *                                units (N.m/T, N.m^2/T, N/T)
     *
     * Details:
     * A magnetic point dipole m located at a position r experiences both
     * a force and a torque in response to the field gradient and field
     * vector, respectively, at the position r. The force and torque can
     * be combined into a single 6x1 entity called a wrench, which has
     * mixed units (N and N.m).
     * A wrench is a screw $ = (f;tO) that describes the instantaneous
     * dynamics of a rigid body. I use the ray-coordinate convention to
     * describe screws. The 3x1 vector f is the force applied to the body.
     * The 3x1 vector tO is the torque applied about an imaginary point on
     * the body located at the origin of the present frame:
     *      tO = t + r x f
     * where t is the torque on the body at a point along the line of
     * action of the force and r is a vector from the origin of the frame
     * to the line of action.
     * For more details, see Davidson & Hunt, Robots and Screw Theory,
     * 2004.
     * Using wrench notation allows a linear expression to be derived that
     * expresses the wrench applied to a magnetic dipole exposed to an
     * 8x1 augmented field vector beta = [b;g]:
     *      $ = Mw * beta
     */
    Eigen::Matrix<double,6,8> Mw;
    Eigen::Matrix3d Mt = calc_dipole_torque_matrix(dipoleMoment);
    Eigen::Matrix<double,3,5> Mf = calc_dipole_force_matrix(dipoleMoment);
    Mw << Eigen::Matrix3d::Zero(),                         Mf,
                               Mt, calc_xprod_skew_mat(position)*Mf;
    return Mw;
}

Eigen::Matrix3d magF::calc_xprod_skew_mat(Eigen::Vector3d v)
{
    /*
     * This function returns the skew-symmetric matrix of a vector v that
     * is equivalent to the cross product.
     *
     * Inputs:
     * Eigen::Vector3d v - 3x1 vector
     *
     * Outputs:
     * Eigen::Matrix3d M - 3x3 matrix
     *
     * Details:
     * The cross product between two 3x1 vectors is usually represented as:
     *      u = v x w
     * However, it can be convenient to express the cross product as a
     * matrix multiplication:
     *      u = M(v)w
     * Where M is a 3x3 skew-symmetric matrix:
     *      M = [  0 -v3  v2]
     *          [ v3   0 -v1]
     *          [-v2  v1   0]
     * See p. 59 Abbott et al., "Magnetic Methods in Robotics," Ann. Rev.
     * Ctrl. Robot. Auton. Syst., vol. 3, pp. 57-90, 2020.
     */
    Eigen::Matrix3d M;
    M <<     0, -v(2),  v(1),
          v(2),     0, -v(0),
         -v(1),  v(0),     0;
    return M;
}



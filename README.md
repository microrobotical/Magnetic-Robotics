
# Magnetic-Robotics
Useful function and class definitions for applications in magnetic robotics

# Dependencies
[Eigen 3.4.0](https://eigen.tuxfamily.org)

# Conventions
Several conventions are employed in this project. The choice of these conventions is often arbitrary and may not be obvious on initial use. The code contains comments documenting these conventions where they occur, but a brief summary is included here as well.

## Denavit-Hartenberg Convention
The Denavit-Hartenberg (DH) convention is a standardized way of expressing the geometry of rigid-link robots. This code uses the standard (not modified) DH parameters: the *i*^th^ joint precedes (is proximal to) the *i*^th^ link, and the *z*-axis of the *(i-1)*^th^ frame is collinear with the rotational or translational axis of the *i*^th^ joint. The *i*^th^ frame is rigidly attached to the *i*^th^ link.
`Link 0 (Base Link)` -> `Joint 1 (Frame 0)` -> `Link 1` -> `Joint 2 (Frame 1)` -> `Link 2` -> ... -> `Joint n (Frame n-1)` -> `Link n (End Effector)` -> `Frame n (EE frame)`

## Screws
The mathematical object called a "screw" is a useful concept in robotics because it enables a compact and generalized way of representing instantaneous rigid-body kinematics and dynamics. This code uses the ray-coordinate order of screw objects: *$ = (L, M, N; P, Q, R)*.

## Augmented Magnetic Field
The force and torque on a magnetic point dipole located at a point in space in a magnetic field is a function of the field vector and spatial gradient at that point in space. In most cases, physical constraints on the divergence and curl of the magnetic field result in only five of the nine spatial gradients being independent. As a result, we can define an augmented 8x1 magnetic field vector
*beta* = [*b*; *g*],
where *b* = [*b~x~*; *b~y~*; *b~z~*] is the 3x1 field vector at the point in space and *g* is a 5x1 vector containing selected spatial gradients of the magnetic field. By convention, this code uses the following spatial gradients in the following order:
*g* = [*g~xx~*; *g~xy~*; *g~xz~*; *g~yy~*; *g~yz~*]
where *g~xy~* denotes the partial derivative of *b~x~* with respect to *y*.

# Details

## `magfunctions.h` and `magfunctions.cpp`
These files contain useful magnetic physics functions and constants. 

The functions are organized into the `magF` namespace to avoid potential clashes with other functions. So, for example, the value of the permeability of free space can be accessed with `magF::mu0`.

## `robotfunctions.h` and `robotfunctions.cpp`
These files contain some useful robotics functions. In particular, the `serialRobot` class is defined here.

The functions and classes are organizes into the `robF` namespace to avoid potential clashes with other functions.

The `serialRobot` class is defined here, which implements functionality allowing robot objects to be defined with DH parameters. Once a robot object is instantiated, its DH parameter members cannot be modified directly (because they are protected members). Instead, the configuration (generalized coordinates) of the robot can be returned using the `getq()` function member and changed using the `setq(q)` function member.

## The `magSerialRobot` Class
The `magSerialRobot` class is dependent on the `robotfunctions.h` and `magfunctions.h` files. It is a derived class from the `serialRobot` class defined in `robotfunctions.h`, so it inherits the DH-related functionality of that class but also includes members that define the magnitudes, directions, and positions of the point dipoles representing the magnetic material in the robot links.

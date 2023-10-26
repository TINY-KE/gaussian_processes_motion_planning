import numpy as np
from gtsam import *
from gpmp2 import *
import math


def generateMobileArm(inp_str, base_T_arm=None):
    # %GENERATEMOBILEARM Generate mobile arm model
    # %
    # %   Usage: model = GENERATEMOBILEARM(arm_str)
    # %   @str        dataset string, existing datasets:
    # %               'SimpleTwoLinksArm'
    # %
    # %   Output Format:
    # %   model       an ArmModel object, contains kinematics and model information

    if base_T_arm is None:
        base_pose = Pose3(Rot3(np.identity(3)), Point3(np.asarray([0, 0, 0])))

    #%  2 link arm
    if inp_str is "SimpleTwoLinksArm":
        #% abstract arm
        a = np.asarray([0.3, 0.3])
        d = np.asarray([0, 0])
        alpha = np.asarray([0, 0])
        arm = Arm(2, a, alpha, d)
        #% abstract mobile arm
        marm = Pose2MobileArm(arm)
        #% physical model
        #% base size: 0.4 x 0.2
        spheres_data = [
            [0, -0.1, 0.0, 0.0, 0.12],
            [0, 0.0, 0.0, 0.0, 0.12],
            [0, 0.1, 0.0, 0.0, 0.12],
            [1, -0.3, 0.0, 0.0, 0.05],
            [1, -0.2, 0.0, 0.0, 0.05],
            [1, -0.1, 0.0, 0.0, 0.05],
            [2, -0.3, 0.0, 0.0, 0.05],
            [2, -0.2, 0.0, 0.0, 0.05],
            [2, -0.1, 0.0, 0.0, 0.05],
            [2, 0.0, 0.0, 0.0, 0.05],
        ]
        spheres_data = np.asarray(spheres_data)
        nr_body = spheres_data.shape[0]
        sphere_vec = BodySphereVector()
        for i in range(nr_body):
            sphere_vec.push_back(
                BodySphere(
                    spheres_data[i, 0], spheres_data[i, 4], Point3(spheres_data[i, 1:4])
                )
            )
        model = Pose2MobileArmModel(marm, sphere_vec)

    #%  2 simple arms on mobile base, each arm has two links
    elif inp_str is "SimpleTwoArms":
        #% abstract arm
        a = np.asarray([0.6, 0.6])
        d = np.asarray([0, 0])
        alpha = np.asarray([0, 0])
        arm = Arm(2, a, alpha, d)
        #% abstract mobile arm
        #% base pose to left and right 60 deg
        marm = Pose2Mobile2Arms(
            arm,
            arm,
            Pose3(Rot3.Yaw(-np.pi / 3), Point3(0, 0, 0)),
            Pose3(Rot3.Yaw(np.pi / 3), Point3(0, 0, 0)),
        )
        #% physical model
        #% base size: 0.8 x 0.4
        spheres_data = [
            [0, -0.2, 0.0, 0.0, 0.24],
            [0, 0.0, 0.0, 0.0, 0.24],
            [0, 0.2, 0.0, 0.0, 0.24],
            [1, -0.6, 0.0, 0.0, 0.1],
            [1, -0.4, 0.0, 0.0, 0.1],
            [1, -0.2, 0.0, 0.0, 0.1],
            [2, -0.6, 0.0, 0.0, 0.1],
            [2, -0.4, 0.0, 0.0, 0.1],
            [2, -0.2, 0.0, 0.0, 0.1],
            [2, 0.0, 0.0, 0.0, 0.1],
            [3, -0.6, 0.0, 0.0, 0.1],
            [3, -0.4, 0.0, 0.0, 0.1],
            [3, -0.2, 0.0, 0.0, 0.1],
            [4, -0.6, 0.0, 0.0, 0.1],
            [4, -0.4, 0.0, 0.0, 0.1],
            [4, -0.2, 0.0, 0.0, 0.1],
            [4, 0.0, 0.0, 0.0, 0.1],
        ]
        spheres_data = np.asarray(spheres_data)
        nr_body = spheres_data.shape[0]
        sphere_vec = BodySphereVector()
        for i in range(nr_body):
            sphere_vec.push_back(
                BodySphere(
                    spheres_data[i, 0], spheres_data[i, 4], Point3(spheres_data[i, 1:4])
                )
            )
        model = Pose2MobileArmModel(marm, sphere_vec)

    #%  2DMobileArm2
    elif inp_str is "2DMobileArm2":
        a = np.asarray([1.0, 1.0])
        d = np.asarray([0, 0])
        alpha = np.asarray([0, 0])
        arm = Arm(2, a, alpha, d)
        #% abstract mobile arm
        marm = Pose2MobileArm(arm)
        #% physical model
        #% base size: 1.0 x 0.7
        spheres_data = [
            [0, 0.2, 0.0, 0.0, 0.35],
            [0, -0.2, 0.0, 0.0, 0.35],
            [1, -0.05, 0.0, 0.0, 0.1],
            [1, -0.25, 0.0, 0.0, 0.1],
            [1, -0.45, 0.0, 0.0, 0.1],
            [2, -0.05, 0.0, 0.0, 0.1],
            [2, -0.25, 0.0, 0.0, 0.1],
            [2, -0.45, 0.0, 0.0, 0.1],
            [2, -0.65, 0.0, 0.0, 0.1],
            [2, -0.85, 0.0, 0.0, 0.1],
        ]
        spheres_data = np.asarray(spheres_data)
        nr_body = spheres_data.shape[0]
        sphere_vec = BodySphereVector()
        for i in range(nr_body):
            sphere_vec.push_back(
                BodySphere(
                    spheres_data[i, 0], spheres_data[i, 4], Point3(spheres_data[i, 1:4])
                )
            )
        model = Pose2MobileArmModel(marm, sphere_vec)

    #%  Vector: omni drive base with 6 DOF Kinova JACO2 arm
    elif inp_str is "Vector":
        #% arm: JACO2 6DOF arm
        alpha = np.asarray([np.pi / 2, np.pi, np.pi / 2, 1.0472, 1.0472, np.pi])
        a = np.asarray([0, 0.41, 0, 0, 0, 0])
        d = np.asarray([0.2755, 0, -0.0098, -0.2501, -0.0856, -0.2228])
        #% theta = [0, 0, 0, 0, 0, 0]';
        #% abstract arm
        arm = Arm(6, a, alpha, d)
        #% abstract mobile arm
        marm = Pose2MobileArm(arm, base_T_arm)
        #% sphere data [id x y z r]
        spheres_data = [
            [0, -0.01, 0, 0, 0.005],
            [0, -0.26, -0.01, 0.08, 0.08],
            [0, -0.26, 0.15, 0.08, 0.08],
            [0, -0.26, -0.17, 0.08, 0.08],
            [0, 0.24, -0.01, 0.08, 0.08],
            [0, 0.24, 0.15, 0.08, 0.08],
            [0, 0.24, -0.17, 0.08, 0.08],
            [0, 0.04, -0.01, 0.6, 0.18],
            [0, -0.2, -0.06, 0.45, 0.1],
            [0, -0.2, 0.04, 0.45, 0.1],
            [0, 0.16, -0.07, 0.41, 0.06],
            [0, 0.16, 0.05, 0.41, 0.06],
            [0, 0.16, -0.18, 0.41, 0.06],
            [0, 0.16, 0.16, 0.41, 0.06],
            [0, 0.33, -0.01, 0.29, 0.05],
            [0, -0.01, -0.24, 0.31, 0.05],
            [0, -0.12, -0.24, 0.31, 0.05],
            [0, -0.22, -0.24, 0.31, 0.05],
            [0, -0.32, -0.24, 0.31, 0.05],
            [0, 0.1, -0.24, 0.31, 0.05],
            [0, 0.2, -0.24, 0.31, 0.05],
            [0, 0.3, -0.24, 0.31, 0.05],
            [0, -0.01, 0.22, 0.31, 0.05],
            [0, -0.12, 0.22, 0.31, 0.05],
            [0, -0.22, 0.22, 0.31, 0.05],
            [0, -0.32, 0.22, 0.31, 0.05],
            [0, 0.1, 0.22, 0.31, 0.05],
            [0, 0.2, 0.22, 0.31, 0.05],
            [0, 0.3, 0.22, 0.31, 0.05],
            [0, -0.32, -0.01, 0.31, 0.05],
            [0, -0.32, 0.10, 0.31, 0.05],
            [0, -0.32, -0.13, 0.31, 0.05],
            [0, 0.32, -0.01, 0.31, 0.05],
            [0, 0.32, 0.10, 0.31, 0.05],
            [0, 0.32, -0.13, 0.31, 0.05],
            [0, 0.12, -0.01, 0.87, 0.1],
            [0, 0.14, -0.11, 0.78, 0.08],
            [0, 0.14, 0.09, 0.78, 0.08],
            [0, 0.19, -0.01, 1.07, 0.08],
            [0, 0.14, -0.11, 0.97, 0.08],
            [0, 0.14, 0.09, 0.97, 0.08],
            [0, 0.175, -0.01, 1.2, 0.05],
            [0, 0.175, -0.01, 1.3, 0.05],
            [0, 0.175, -0.01, 1.4, 0.05],
            [0, 0.175, -0.01, 1.5, 0.05],
            [0, 0.175, -0.01, 1.62, 0.07],
            [0, 0.27, -0.01, 1.5, 0.05],
            [0, 0.37, -0.01, 1.5, 0.05],
            [0, 0.37, -0.01, 1.6, 0.05],
            [0, 0.37, -0.01, 1.66, 0.045],
            [0, 0.37, -0.1, 1.66, 0.045],
            [0, 0.37, 0.08, 1.66, 0.045],
            [1, 0.0, 0.0, 0.0, 0.053],
            [1, 0.0, -0.08, 0.0, 0.053],
            [1, 0.0, -0.155, 0.0, 0.053],
            [1, 0.0, -0.23, 0.0, 0.053],
            [2, 0.0, 0.0, 0.0, 0.053],
            [2, -0.06, 0.0, 0.03, 0.04],
            [2, -0.12, 0.0, 0.03, 0.04],
            [2, -0.18, 0.0, 0.03, 0.04],
            [2, -0.24, 0.0, 0.03, 0.04],
            [2, -0.30, 0.0, 0.03, 0.04],
            [2, -0.36, 0.0, 0.03, 0.04],
            [3, 0.0, -0.01, -0.05, 0.035],
            [3, 0.0, -0.01, -0.10, 0.03],
            [3, 0.0, 0.0, -0.15, 0.035],
            [3, 0.0, 0.0, -0.2, 0.035],
            [4, 0.0, 0.0, 0.0, 0.04],
            [4, 0.0, 0.0, -0.045, 0.04],
            [5, 0.0, 0.0, 0.0, 0.04],
            [5, 0.0, -0.008, -0.075, 0.05],
            [6, 0.0, 0.05, -0.01, 0.013],
            [6, 0.0, 0.05, 0.01, 0.013],
            [6, 0.0, 0.06, -0.039, 0.018],
            [6, 0.0, 0.06, -0.067, 0.018],
            [6, 0.0, 0.035, -0.042, 0.018],
            [6, 0.0, -0.05, -0.01, 0.013],
            [6, 0.0, -0.05, 0.01, 0.013],
            [6, 0.0, -0.06, -0.039, 0.018],
            [6, 0.0, -0.06, -0.067, 0.018],
            [6, 0.0, -0.035, -0.042, 0.018],
            [6, 0.0, 0.015, -0.055, 0.02],
            [6, 0.0, 0.025, -0.08, 0.02],
            [6, 0.0, 0.0, -0.08, 0.02],
            [6, 0.0, -0.025, -0.08, 0.02],
            [6, 0.0, -0.015, -0.055, 0.02],
        ]

        spheres_data = np.asarray(spheres_data)
        nr_body = spheres_data.shape[0]
        sphere_vec = BodySphereVector()
        for i in range(nr_body):
            sphere_vec.push_back(
                BodySphere(
                    spheres_data[i, 0], spheres_data[i, 4], Point3(spheres_data[i, 1:4])
                )
            )
        model = Pose2MobileArmModel(marm, sphere_vec)

    #%  PR2: 3 DOF base + 1 DOF linear actuator + 2 x 7 DOF arms
    elif inp_str is "PR2":
        #% abstract arm
        alpha = np.asarray([-1.5708, 1.5708, -1.5708, 1.5708, -1.5708, 1.5708, 0])
        a = np.asarray(np.asarray([0.1, 0, 0, 0, 0, 0, 0]))
        d = np.asarray([0, 0, 0.4, 0, 0.321, 0, 0])
        theta = np.asarray([0, 1.5708, 0, 0, 0, 0, 0])
        arm = Arm(7, a, alpha, d, Pose3(), theta)

        base_T_torso = Pose3(Rot3(), Point3(np.asarray([-0.05, 0, 0.790675])))
        torso_T_arm_l = Pose3(Rot3(), Point3(np.asarray([0, 0.188, 0])))
        torso_T_arm_r = Pose3(Rot3(), Point3(np.asarray([0, -0.188, 0])))
        marm = Pose2MobileVetLin2Arms(
            arm, arm, base_T_torso, torso_T_arm_l, torso_T_arm_r, false
        )

        spheres_data = [
            [0, 0.000000, 0.000000, 0.130000, 0.170000],
            [0, 0.230000, 0.000000, 0.130000, 0.170000],
            [0, -0.230000, 0.000000, 0.130000, 0.170000],
            [0, 0.230000, 0.230000, 0.130000, 0.170000],
            [0, 0.000000, 0.230000, 0.130000, 0.170000],
            [0, 0.000000, -0.230000, 0.130000, 0.170000],
            [0, 0.230000, -0.230000, 0.130000, 0.170000],
            [0, -0.230000, -0.230000, 0.130000, 0.170000],
            [0, -0.230000, 0.230000, 0.130000, 0.170000],
            [0, -0.27000, 0.00000, 0.380000, 0.080000],
            [0, -0.27000, 0.16000, 0.380000, 0.080000],
            [0, -0.27000, -0.16000, 0.380000, 0.080000],
            [0, -0.27000, 0.00000, 0.540000, 0.080000],
            [0, -0.27000, 0.14000, 0.540000, 0.080000],
            [0, -0.27000, -0.14000, 0.540000, 0.080000],
            [1, -0.110000, 0.000000, 0.100000, 0.250000],
            [1, -0.090000, -0.120000, -0.340000, 0.200000],
            [1, -0.090000, 0.120000, -0.340000, 0.200000],
            [1, -0.020000, 0.000000, 0.370000, 0.170000],
            [2, -0.010000, 0.000000, 0.000000, 0.180000],
            [4, 0.015000, 0.220000, -0.000000, 0.110000],
            [4, 0.035000, 0.140000, -0.000000, 0.080000],
            [4, 0.035000, 0.072500, -0.000000, 0.080000],
            [4, 0.000000, 0.000000, -0.000000, 0.105000],
            [6, -0.005000, 0.321 - 0.130000, -0.000000, 0.075000],
            [6, 0.010000, 0.321 - 0.200000, -0.025000, 0.055000],
            [6, 0.010000, 0.321 - 0.200000, 0.025000, 0.055000],
            [6, 0.015000, 0.321 - 0.265000, -0.027500, 0.050000],
            [6, 0.015000, 0.321 - 0.265000, 0.027500, 0.050000],
            [6, 0.005000, 0.321 - 0.320000, -0.022500, 0.050000],
            [6, 0.005000, 0.321 - 0.320000, 0.022500, 0.050000],
            [8, 0, -0.017500, 0.072500, 0.040000],
            [8, 0, 0.017500, 0.072500, 0.040000],
            [8, 0, 0, 0.092500, 0.040000],
            [8, 0, 0.03600, 0.11, 0.040000],
            [8, 0, 0.027000, 0.155, 0.035000],
            [8, 0, 0.00900, 0.18, 0.030000],
            [8, 0, 0.00950, 0.205, 0.020000],
            [8, 0, -0.03600, 0.11, 0.040000],
            [8, 0, -0.027000, 0.155, 0.035000],
            [8, 0, -0.00900, 0.18, 0.030000],
            [8, 0, -0.00950, 0.205, 0.020000],
            [9, -0.010000, 0.000000, 0.000000, 0.180000],
            [11, 0.015000, 0.220000, -0.000000, 0.110000],
            [11, 0.035000, 0.140000, -0.000000, 0.080000],
            [11, 0.035000, 0.072500, -0.000000, 0.080000],
            [11, 0.000000, 0.000000, -0.000000, 0.105000],
            [13, -0.005000, 0.321 - 0.130000, -0.000000, 0.075000],
            [13, 0.010000, 0.321 - 0.200000, -0.025000, 0.055000],
            [13, 0.010000, 0.321 - 0.200000, 0.025000, 0.055000],
            [13, 0.015000, 0.321 - 0.265000, -0.027500, 0.050000],
            [13, 0.015000, 0.321 - 0.265000, 0.027500, 0.050000],
            [13, 0.005000, 0.321 - 0.320000, -0.022500, 0.050000],
            [13, 0.005000, 0.321 - 0.320000, 0.022500, 0.050000],
            [15, 0, -0.017500, 0.072500, 0.040000],
            [15, 0, 0.017500, 0.072500, 0.040000],
            [15, 0, 0, 0.092500, 0.040000],
            [15, 0, 0.03600, 0.11, 0.040000],
            [15, 0, 0.027000, 0.155, 0.035000],
            [15, 0, 0.00900, 0.18, 0.030000],
            [15, 0, 0.00950, 0.205, 0.020000],
            [15, 0, -0.03600, 0.11, 0.040000],
            [15, 0, -0.027000, 0.155, 0.035000],
            [15, 0, -0.00900, 0.18, 0.030000],
            [15, 0, -0.00950, 0.205, 0.020000],
        ]
        spheres_data = np.asarray(spheres_data)
        nr_body = spheres_data.shape[0]
        sphere_vec = BodySphereVector()
        for i in range(nr_body):
            sphere_vec.push_back(
                BodySphere(
                    spheres_data[i, 0], spheres_data[i, 4], Point3(spheres_data[i, 1:4])
                )
            )
        model = Pose2MobileArmModel(marm, sphere_vec)

    #% no such dataset
    else:
        raise NameError("No such arm exists")
    return model

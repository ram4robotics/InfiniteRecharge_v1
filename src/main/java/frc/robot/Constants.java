/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int kLeftMotor1_id = 11;
        public static final int kLeftMotor2_id = 12;
        public static final int kRightMotor1_id = 13;
        public static final int kRightMotor2_id = 14;

        public static final double kDriveGearRatio = 10.71;

        public static final int kSparkMaxBuiltinCPR = 42;
        public static final int kEncoderCPR = kSparkMaxBuiltinCPR;
        public static final double kWheelDiameterInches = 6;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are NOT directly mounted on the wheel shafts
            (kWheelDiameterInches * Math.PI) / (double) (kEncoderCPR * kDriveGearRatio);
    }
}

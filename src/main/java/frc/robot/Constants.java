/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int kLeftMotor1_id = 11;
        public static final int kLeftMotor2_id = 12;
        public static final int kRightMotor1_id = 13;
        public static final int kRightMotor2_id = 14;

        public static final double kDriveGearRatio = 10.71;

        public static final int kSparkMaxBuiltinCPR = 42;
        // public static final int kEncoderCPR = kSparkMaxBuiltinCPR;
        // public static final double kWheelDiameterInches = 6;
        // public static final double kEncoderDistancePerPulse =
        //     // Assumes the encoders are NOT directly mounted on the wheel shafts
        //     (kWheelDiameterInches * Math.PI) / (double) (kEncoderCPR * kDriveGearRatio);

        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final int[] kLeftEncoderPorts = new int[]{0, 1};
        public static final int[] kRightEncoderPorts = new int[]{2, 3};
        public static final boolean kLeftEncoderReversed = true;
        public static final boolean kRightEncoderReversed = false;
        
        public static final double kWheelDiameterMeters = 0.1524;
        public static final double kLeftEncoderPulsesPerRev = 8192; // Rev Throughbore encoder
        public static final double kRightEncoderPulsesPerRev = 8192; // Rev Throughbore encoder
        public static final double kLeftMetersPerPulse = Math.PI * kWheelDiameterMeters / kLeftEncoderPulsesPerRev;
        public static final double kRightMetersPerPulse = Math.PI * kWheelDiameterMeters / kRightEncoderPulsesPerRev;
    
        public static final int kNeoEncoderPulsesPerRev = kSparkMaxBuiltinCPR * 4;
        public static final double kNeoEncoderMetersPerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) (kNeoEncoderPulsesPerRev * kDriveGearRatio);
    
        public static final boolean kGyroReversed = true;
    
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    
        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;
    }
    public static final class OIConstants {
        public static final int kDriverControllerPort = 1;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

}

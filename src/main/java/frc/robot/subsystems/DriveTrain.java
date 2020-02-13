/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public class DriveTrain extends SubsystemBase {
  CANSparkMax m_leftMaster, m_leftSlave, m_rightMaster, m_rightSlave;
  CANEncoder  neo_leftEncoder, neo_rightEncoder;
  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  Supplier<Double> leftEncoderPosition;
  Supplier<Double> leftEncoderRate;
  Supplier<Double> rightEncoderPosition;
  Supplier<Double> rightEncoderRate;
  Supplier<Double> gyroAngleRadians;
  DifferentialDrive m_dDrive;
  AHRS m_navx;

  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    m_leftMaster = new CANSparkMax(DriveConstants.kLeftMotor1_id, MotorType.kBrushless);
    m_leftMaster.setInverted(false);
    m_leftMaster.setIdleMode(IdleMode.kBrake);
    neo_leftEncoder = m_leftMaster.getEncoder();

    m_leftSlave = new CANSparkMax(DriveConstants.kLeftMotor2_id, MotorType.kBrushless);
    m_leftSlave.setIdleMode(IdleMode.kBrake);
    m_leftSlave.follow(m_leftMaster);

    m_rightMaster = new CANSparkMax(DriveConstants.kRightMotor1_id, MotorType.kBrushless);
    m_rightMaster.setInverted(false);
    m_rightMaster.setIdleMode(IdleMode.kBrake);
    neo_rightEncoder = m_rightMaster.getEncoder();

    m_rightSlave = new CANSparkMax(DriveConstants.kRightMotor2_id, MotorType.kBrushless);
    m_rightSlave.setIdleMode(IdleMode.kBrake);
    m_rightSlave.follow(m_rightMaster);

    m_dDrive = new DifferentialDrive(m_leftMaster, m_rightMaster);
    m_dDrive.setDeadband(0);

    // The left-side drive encoder
    m_leftEncoder = new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1],
                DriveConstants.kLeftEncoderReversed);
    m_leftEncoder.setDistancePerPulse(DriveConstants.kLeftMetersPerPulse);

    // The right-side drive encoder
    m_rightEncoder = new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1],
                  DriveConstants.kRightEncoderReversed);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kRightMetersPerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    //
    // Configure gyro
    //

    // Note that the angle from the NavX and all implementors of wpilib Gyro
    // must be negated because getAngle returns a clockwise positive angle
    m_navx = new AHRS(SPI.Port.kMXP);
    gyroAngleRadians = () -> -1 * Math.toRadians(m_navx.getAngle());

    // Let's name the sensors on the LiveWindow
    addChild("Drive", m_dDrive);
    addChild("Left Encoder", m_leftEncoder);
    addChild("Right Encoder", m_rightEncoder);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getDistance(),
                      m_rightEncoder.getDistance());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_dDrive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    // ToDo:  Verify whether setVolatge() is applied to the follower motor controller.
    m_leftMaster.setVoltage(leftVolts);
    m_rightMaster.setVoltage(-rightVolts);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_dDrive.setMaxOutput(maxOutput);
  }


  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_navx.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_navx.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_navx.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

}

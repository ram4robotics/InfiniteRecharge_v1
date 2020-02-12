/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

public class Launcher extends SubsystemBase {
  private final CANSparkMax m_motorL, m_motorR;
  private final CANEncoder  m_encoderL, m_encoderR;
  private final CANPIDController m_pidVelocity;
  /**
   * Creates a new Launcher.
   */
  public Launcher() {
    m_motorL = new CANSparkMax(LauncherConstants.kLauncherMotorLeft_id, MotorType.kBrushless);
    m_motorR = new CANSparkMax(LauncherConstants.kLauncherMotorRight_id, MotorType.kBrushless);
    m_motorR.follow(m_motorL, true); // Set the output of right motor to opposite of that of the left motor

    m_motorL.restoreFactoryDefaults();
    m_motorR.restoreFactoryDefaults();
    m_motorL.setClosedLoopRampRate(LauncherConstants.kClosedLoopRampRate);
    m_motorR.setClosedLoopRampRate(LauncherConstants.kClosedLoopRampRate);
    m_motorL.setIdleMode(LauncherConstants.kIdleMode);
    m_motorR.setIdleMode(LauncherConstants.kIdleMode);

    m_encoderL = new CANEncoder(m_motorL);
    m_encoderR = new CANEncoder(m_motorR);
    
    // Setup the controller on leader; the follower will get the voltage values from leader.
    m_pidVelocity = new CANPIDController(m_motorL);

    // set PID coefficients
    m_pidVelocity.setP(LauncherConstants.kP);
    m_pidVelocity.setI(LauncherConstants.kI);
    m_pidVelocity.setD(LauncherConstants.kD);
    m_pidVelocity.setIZone(LauncherConstants.kIz);
    m_pidVelocity.setFF(LauncherConstants.kFF);
    m_pidVelocity.setOutputRange(LauncherConstants.kMinOutput, LauncherConstants.kMaxOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void launch(double pctSpeed) {
    double setPoint = pctSpeed * LauncherConstants.maxRPM;
    m_pidVelocity.setReference(setPoint, ControlType.kVelocity);
    // m_motorL.set(LauncherConstants.kLauncherPower);
  }
}

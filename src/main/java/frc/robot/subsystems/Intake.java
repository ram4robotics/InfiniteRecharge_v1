/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Intake extends SubsystemBase {
  private final TalonSRX m_motor;
  private final Encoder m_Encoder;
  private final DoubleSolenoid m_sol;
  /**
   * Creates a new Intake.
   */
  public Intake() {
    m_motor = new TalonSRX(IntakeConstants.kIntakeMotor_id);
    m_motor.configFactoryDefault();
    m_motor.setNeutralMode(NeutralMode.Brake);
    m_motor.setInverted(IntakeConstants.kMotorInverted);

    m_Encoder = new Encoder(IntakeConstants.kEncoderPorts[0], IntakeConstants.kEncoderPorts[1]);
    m_Encoder.setReverseDirection(IntakeConstants.kEncoderReversed);
    resetEncoder();

    m_sol = new DoubleSolenoid(IntakeConstants.kDSolenoidPorts[0], IntakeConstants.kDSolenoidPorts[1]);

    addChild("Intake Encoder", m_Encoder);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoder() {
    m_Encoder.reset();
  }

  public void pullInPowerCells() {
    // ToDo:  Need to add Velocity closed loop controller based on 
    // either the frc-characterization output or CANifier-connected throughbore encoder.
    m_motor.set(ControlMode.PercentOutput, IntakeConstants.kPullInPower);
  }

  public void pushOutPowerCells() {
    // ToDo:  Need to add Velocity closed loop controller based on 
    // either the frc-characterization output or CANifier-connected throughbore encoder.
    m_motor.set(ControlMode.PercentOutput, IntakeConstants.kPushOutPower);
  }

  public void stopIntake() {
    m_motor.set(ControlMode.PercentOutput, 0.0);
  }

  public void foldArm() {
    m_sol.set(Value.kReverse);
  }

  public void unfoldArm() {
    m_sol.set(Value.kForward);
  }
}

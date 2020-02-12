/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
  private final VictorSPX m_motor;
  private final Encoder  m_Encoder;
  /**
   * Creates a new Indexer.
   */
  public Indexer() {
    m_motor = new VictorSPX(IndexerConstants.kIndexerMotor_id);
    m_motor.configFactoryDefault();
    m_motor.setNeutralMode(NeutralMode.Brake);
    m_motor.setInverted(IndexerConstants.kMotorInverted);

    m_Encoder = new Encoder(IndexerConstants.kEncoderPorts[0], IndexerConstants.kEncoderPorts[1]);
    m_Encoder.setReverseDirection(IndexerConstants.kEncoderReversed);
    resetEncoder();

    addChild("Indexer Encoder", m_Encoder);
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
    m_motor.set(ControlMode.PercentOutput, IndexerConstants.kPullInPower);
  }

  public void pushOutPowerCells() {
    // ToDo:  Need to add Velocity closed loop controller based on 
    // either the frc-characterization output or CANifier-connected throughbore encoder.
    m_motor.set(ControlMode.PercentOutput, IndexerConstants.kPushOutPower);
  }
}

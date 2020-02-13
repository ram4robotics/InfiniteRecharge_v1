/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoPathsConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

public class Auto_pos3_path1 extends CommandBase {
  private DriveTrain m_drvTrain;
  private Intake m_intake;
  private Indexer m_indexer;
  private Launcher m_launcher;

  /**
   * Creates a new Auto_pos3_path1.
   */
  public Auto_pos3_path1(DriveTrain drvTrain, Intake intake, Indexer indexer, Launcher launcher) {
    m_drvTrain = drvTrain;
    m_intake = intake;
    m_indexer = indexer;
    m_launcher = launcher;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drvTrain);
    addRequirements(intake);
    addRequirements(indexer);
    addRequirements(launcher);
  }

  public Command getAutoCommand() {
    // An ExampleCommand will run in autonomous
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                       DriveConstants.kvVoltSecondsPerMeter,
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    RamseteCommand ramseteCommands[] = new RamseteCommand[AutoPathsConstants.kPos3Path1.length];
    for (int i=0; i<ramseteCommands.length; i++) {
      String trajectoryJSON = AutoPathsConstants.kPos3Path1[i];
      // String trajectoryJSON = "paths/Auto_pos3_path1.json";
      Path trajectoryPath;
      Trajectory exampleTrajectory;
      try {
        trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        // An example trajectory to follow.  All units in meters.
        exampleTrajectory = TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(
              new Translation2d(1, 1),
              new Translation2d(2, -1)
          ),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(3, 0, new Rotation2d(0)),
          // Pass config
          config
        );
      }

      ramseteCommands[i] = new RamseteCommand(
          exampleTrajectory,
          m_drvTrain::getPose,
          new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
          new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                    DriveConstants.kvVoltSecondsPerMeter,
                                    DriveConstants.kaVoltSecondsSquaredPerMeter),
          DriveConstants.kDriveKinematics,
          m_drvTrain::getWheelSpeeds,
          new PIDController(DriveConstants.kPDriveVel, 0, 0),
          new PIDController(DriveConstants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          m_drvTrain::tankDriveVolts,
          m_drvTrain
      );

    }

    // Run path following command, then stop at the end.
    Command finalAutoCommand = new SequentialCommandGroup(
      ramseteCommands[0].andThen(() -> m_launcher.launch(0.9)),
      ramseteCommands[1],
      ramseteCommands[2].alongWith(new PullInPowerCells(m_intake)),
      ramseteCommands[3].andThen(() -> m_launcher.launch(0.9)));

    return (finalAutoCommand.andThen(() -> m_drvTrain.tankDriveVolts(0, 0)));
    // return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

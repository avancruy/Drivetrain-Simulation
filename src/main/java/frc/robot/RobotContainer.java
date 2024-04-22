// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathPlanning;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  private final Drivetrain drivetrain = new Drivetrain();

  private final CommandXboxController driver = new CommandXboxController(0); 

  public RobotContainer() {
    configureBindings();

    drivetrain.setDefaultCommand(
      new RunCommand(
        () -> drivetrain.arcadeDrive(
          -driver.getLeftY(), -driver.getRightX()), 
          drivetrain));
  }

  private void configureBindings() {
    driver.y().onTrue(new InstantCommand(() -> drivetrain.resetOdometry(DriveConstants.blueSubWooferAmpSide)).ignoringDisable(true));
    driver.b().onTrue(new InstantCommand(() -> drivetrain.resetOdometry(DriveConstants.blueSubWooferCentre)).ignoringDisable(true));
    driver.a().onTrue(new InstantCommand(() -> drivetrain.resetOdometry(DriveConstants.blueSubWooferSourceSide)).ignoringDisable(true));
  }

  public Command getAutonomousCommand() {

    Trajectory trajectory = PathPlanning.blueAllMidNotes;
    
    RamseteCommand ramseteCommand =
            new RamseteCommand(
                trajectory,
                drivetrain::getPose,
                new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
                new SimpleMotorFeedforward(
                    DriveConstants.ksVolts,
                    DriveConstants.kvVoltSecondsPerMeter,
                    DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
                drivetrain::getWheelSpeeds,
                new PIDController(DriveConstants.kPVel, 0, 0),
                new PIDController(DriveConstants.kPVel, 0, 0),
                // RamseteCommand passes volts to the callback
                drivetrain::tankDriveVolts,
                drivetrain);
    
    Command runTraj = Commands.runOnce(() -> drivetrain.resetOdometry(trajectory.getInitialPose()))
      .andThen(ramseteCommand)
      .andThen(Commands.runOnce(() -> drivetrain.tankDriveVolts(0, 0)));

    return runTraj;
  }
}

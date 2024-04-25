// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class RotateToAngle extends Command {
  private final Drivetrain drivetrain;
  private final PIDController pid;
  private Supplier<Double> angle;
  public RotateToAngle(Drivetrain drivetrain, Supplier<Double> a) {
    this.drivetrain = drivetrain;
    this.pid = new PIDController(0.1, 0.05, 0);
    this.angle = a;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("RotateToAngle Start");
    pid.reset();
    pid.setIZone(5);
    pid.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pid.setSetpoint(angle.get());
    SmartDashboard.putNumber("Setpoint", pid.getSetpoint());

    double leftSpeed = -pid.calculate(drivetrain.getAngle());
    double rightSpeed = pid.calculate(drivetrain.getAngle());

    drivetrain.tankDriveVolts(
      leftSpeed, 
      rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("RotateToAngle End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}

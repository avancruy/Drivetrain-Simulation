// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  private final CANSparkMax frontRight = new CANSparkMax(DriveConstants.frontRightID,  MotorType.kBrushless);
  private final CANSparkMax frontLeft = new CANSparkMax(DriveConstants.frontLeftID,  MotorType.kBrushless);
  private final CANSparkMax backRight = new CANSparkMax(DriveConstants.backRightID,  MotorType.kBrushless);
  private final CANSparkMax backLeft = new CANSparkMax(DriveConstants.backLeftID,  MotorType.kBrushless);

  private final RelativeEncoder frontRightEncoder = frontRight.getEncoder();
  private final RelativeEncoder frontLeftEncoder = frontLeft.getEncoder();
  private final RelativeEncoder backRightEncoder = backRight.getEncoder();
  private final RelativeEncoder backLeftEncoder = backLeft.getEncoder();

  private final Pigeon2 gyro = new Pigeon2(DriveConstants.pigeonID);

  private final DifferentialDrive drive = new DifferentialDrive(frontLeft::set, frontRight::set);
  private final DifferentialDriveOdometry odometry;

  private final DifferentialDrivetrainSim drivetrainSim;
  private final Pigeon2SimState gyroSim;

  private final Field2d gameField = new Field2d();
  
  public Drivetrain() {
    SendableRegistry.addChild(drive, frontLeft);
    SendableRegistry.addChild(drive, frontRight);

    frontRight.restoreFactoryDefaults();
    frontLeft.restoreFactoryDefaults();
    backRight.restoreFactoryDefaults();
    backLeft.restoreFactoryDefaults();

    frontLeft.setInverted(true);
    backLeft.setInverted(true);

    frontRight.setInverted(false);
    backRight.setInverted(false);

    backRight.follow(frontRight);
    backLeft.follow(frontLeft);

    frontLeft.setSmartCurrentLimit(40);
    backLeft.setSmartCurrentLimit(40);
    frontRight.setSmartCurrentLimit(40);
    backRight.setSmartCurrentLimit(40);

    frontRightEncoder.setPositionConversionFactor(DriveConstants.RevToMetre);
    backRightEncoder.setPositionConversionFactor(DriveConstants.RevToMetre);
    frontLeftEncoder.setPositionConversionFactor(DriveConstants.RevToMetre);
    backLeftEncoder.setPositionConversionFactor(DriveConstants.RevToMetre);

    frontRightEncoder.setVelocityConversionFactor(DriveConstants.RPMToMetresPerSecond);
    backRightEncoder.setVelocityConversionFactor(DriveConstants.RPMToMetresPerSecond);
    frontLeftEncoder.setVelocityConversionFactor(DriveConstants.RPMToMetresPerSecond);
    backLeftEncoder.setVelocityConversionFactor(DriveConstants.RPMToMetresPerSecond);

    frontLeft.burnFlash();
    backLeft.burnFlash();
    frontRight.burnFlash();
    backRight.burnFlash();

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition());
    odometry.resetPosition(new Rotation2d(), 0, 0, DriveConstants.blueSubWooferCentre);

    if (RobotBase.isSimulation()) {
      drivetrainSim = new DifferentialDrivetrainSim(
        DCMotor.getNEO(2), 
        10.65, 
        6.386, 
        Units.lbsToKilograms(123), 
        Units.inchesToMeters(6), 
        DriveConstants.kTrackwidthMeters, 
        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

      gyroSim = new Pigeon2SimState(gyro);

      SmartDashboard.putData(gameField);
    } else {
      drivetrainSim = null;
      gyroSim = null;
    }
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(),
      frontLeftEncoder.getPosition(),
      frontRightEncoder.getPosition());

      gameField.setRobotPose(odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    drivetrainSim.setInputs(
      frontLeft.get() * RobotController.getBatteryVoltage(),
      frontRight.get() * RobotController.getBatteryVoltage());

    drivetrainSim.update(0.02);

    frontLeftEncoder.setPosition(drivetrainSim.getLeftPositionMeters());
    frontRightEncoder.setPosition(drivetrainSim.getRightPositionMeters());

    gyroSim.setRawYaw(drivetrainSim.getHeading().getDegrees());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    //frontLeft.setVoltage(leftVolts);
    //frontRight.setVoltage(rightVolts);
    frontLeft.set(leftVolts / RobotController.getBatteryVoltage());
    frontRight.set(rightVolts / RobotController.getBatteryVoltage());
    drive.feed();
  }

  public void arcadeDrive(double speed, double rot) {
    drive.arcadeDrive(speed, rot);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    drivetrainSim.setPose(pose);
    odometry.resetPosition(
        gyro.getRotation2d(), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition(), pose);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(frontLeftEncoder.getVelocity(), frontRightEncoder.getVelocity());
  }
}

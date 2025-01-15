// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {
  private static SwerveDrive swerveInstance = null;
  private SwerveModuleState[] states = new SwerveModuleState[4];
  private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
  private SwerveDriveKinematics kinematics;
  private ChassisSpeeds speeds;
  private Pigeon2 pigeon;
  private SwerveDriveOdometry odometry;
  private SwerveDrivePoseEstimator estimator;

  private Translation2d flLocation = new Translation2d(Constants.DrivetrainConstants.X_POS_MODULE,
      Constants.DrivetrainConstants.Y_POS_MODULE);
  private Translation2d frLocation = new Translation2d(Constants.DrivetrainConstants.X_POS_MODULE,
      -Constants.DrivetrainConstants.Y_POS_MODULE);
  private Translation2d blLocation = new Translation2d(-Constants.DrivetrainConstants.X_POS_MODULE,
      Constants.DrivetrainConstants.Y_POS_MODULE);
  private Translation2d brLocation = new Translation2d(-Constants.DrivetrainConstants.X_POS_MODULE,
      -Constants.DrivetrainConstants.Y_POS_MODULE);

  private Module[] modules = new Module[4];
  private Translation2d[] locations = {flLocation, frLocation, blLocation, brLocation};

  StructArrayPublisher<SwerveModuleState> publisher;

  public SwerveDrive() {

    for (int i = 0; i < 4; i++) {
      states[i] = new SwerveModuleState();
      modulePositions[i] = new SwerveModulePosition();
      modules[i] = new Module(Constants.DrivetrainConstants.driveIDs[i], Constants.DrivetrainConstants.azimuthIDs[i],
          Constants.DrivetrainConstants.cancoderIDs[i], Constants.DrivetrainConstants.drivingRatios[i],
          Constants.DrivetrainConstants.steeringRatios[i]);
    }
    kinematics = new SwerveDriveKinematics(locations);
    speeds = new ChassisSpeeds();
    pigeon = new Pigeon2(Constants.DrivetrainConstants.PIGEON_ID);
    odometry = new SwerveDriveOdometry(kinematics, pigeon.getRotation2d(), modulePositions);
    estimator = new SwerveDrivePoseEstimator(kinematics, pigeon.getRotation2d(), modulePositions,
        odometry.getPoseMeters());

    publisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/MyStates", SwerveModuleState.struct).publish();

    resetEncoders();

  }

  public void driveFieldRelative(ChassisSpeeds chassisSpeeds, boolean fieldRelative, double timestamp) {
    ChassisSpeeds speeds = fieldRelative
        ? ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, pigeon.getRotation2d()),
            timestamp)
        : ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, pigeon.getRotation2d());
    states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DrivetrainConstants.MAX_VELOCITY);
    for (int i = 0; i < 4; i++) {
      SwerveModuleState optimizedState = SwerveModuleState.optimize(states[i],
          Rotation2d.fromDegrees(modules[i].getRotFromEncoderPos()));
      modules[i].setInput(optimizedState.speedMetersPerSecond, optimizedState.angle.getRotations());
    }

  }

  public static SwerveDrive getInstance() {
    if (swerveInstance == null) {
      swerveInstance = new SwerveDrive();
    }
    return swerveInstance;
  }

  public Pose2d getPosition() {
    return odometry.getPoseMeters();
  }

  public Pose2d getPositionVision() {
    return null;
  }

  public void resetEncoders() {
    for (Module module : modules) {
      module.resetEncoder();
    }
  }

  public SwerveModuleState[] getStates() {
    return states;
  }

  public void driveController(double xSpeed, double ySpeed, double rot) {
    // SlewRateLimiter joystickOptimizer = new SlewRateLimiter(2);
    // double xSpeedOptimized = joystickOptimizer.calculate(xSpeed *
    // Constants.DrivetrainConstants.MAX_VELOCITY);
    // double ySpeedOptimized = joystickOptimizer.calculate(ySpeed *
    // Constants.DrivetrainConstants.MAX_VELOCITY);
    // double rotOptimized = joystickOptimizer.calculate(rot *
    // Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY);
    ChassisSpeeds controllerSpeeds = new ChassisSpeeds(xSpeed * 4, ySpeed * 4, rot * 3);
    driveFieldRelative(controllerSpeeds, true, 0.1);
  }

  public void testMotor(double input) {
    modules[0].setTranslation(input);
  }

  public void setMotorsInput(double input) {

    for (Module module : modules) {
      module.setInputToMotor(input);
    }
  }

  @Override
  public void periodic() {
    publisher.set(states);

  }

}

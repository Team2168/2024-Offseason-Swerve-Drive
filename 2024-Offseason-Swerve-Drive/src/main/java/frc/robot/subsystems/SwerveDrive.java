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
  private SwerveModuleState[] states;
  private SwerveModulePosition[] modulePositions;
  private SwerveDriveKinematics kinematics;
  private ChassisSpeeds speeds;
  private Pigeon2 pigeon;
  private SwerveDriveOdometry odometry;
  private SwerveDrivePoseEstimator estimator;
  private double mk4SteeringRatio = 12.8;
  private double mk4DrivingRatio = 8.14;
  private double wcpxSteeringRatio = 13.3714;
  private double wcpxDrivingRatio = 8.10;
  private Module frModule = new Module(Constants.DrivetrainConstants.FR_DRIVE,
      Constants.DrivetrainConstants.FR_AZIMUTH, Constants.DrivetrainConstants.FR_CANCODER_ID, wcpxSteeringRatio,
      wcpxDrivingRatio);
  private Module flModule = new Module(Constants.DrivetrainConstants.FL_DRIVE,
      Constants.DrivetrainConstants.FL_AZIMUTH, Constants.DrivetrainConstants.FL_CANCODER_ID, wcpxSteeringRatio,
      wcpxDrivingRatio);
  private Module blModule = new Module(Constants.DrivetrainConstants.BL_DRIVE,
      Constants.DrivetrainConstants.BL_AZIMUTH, Constants.DrivetrainConstants.BL_CANCODER_ID, mk4SteeringRatio,
      mk4DrivingRatio);
  private Module brModule = new Module(Constants.DrivetrainConstants.BR_DRIVE,
      Constants.DrivetrainConstants.BR_AZIMUTH, Constants.DrivetrainConstants.BR_CANCODER_ID, mk4SteeringRatio,
      mk4DrivingRatio);

  private Translation2d flLocation = new Translation2d(Constants.DrivetrainConstants.FL_X,
      Constants.DrivetrainConstants.FL_Y);
  private Translation2d fRLocation = new Translation2d(Constants.DrivetrainConstants.FR_X,
      Constants.DrivetrainConstants.FR_Y);
  private Translation2d blLocation = new Translation2d(Constants.DrivetrainConstants.BL_X,
      Constants.DrivetrainConstants.BL_Y);
  private Translation2d bRLocation = new Translation2d(Constants.DrivetrainConstants.BR_X,
      Constants.DrivetrainConstants.BR_Y);

  private Module[] modules = { frModule, flModule, blModule, brModule };

  public SwerveDrive() {
    states = new SwerveModuleState[4];
    modulePositions = new SwerveModulePosition[4];
    for (int i = 0; i <= 3; i++) {
      states[i] = new SwerveModuleState();
      modulePositions[i] = new SwerveModulePosition();
    }
    kinematics = new SwerveDriveKinematics(fRLocation, flLocation, blLocation, bRLocation);
    speeds = new ChassisSpeeds();
    pigeon = new Pigeon2(Constants.DrivetrainConstants.PIGEON_ID);
    odometry = new SwerveDriveOdometry(kinematics, pigeon.getRotation2d(), modulePositions);
    estimator = new SwerveDrivePoseEstimator(kinematics, pigeon.getRotation2d(), modulePositions,
        odometry.getPoseMeters());
    resetEncoders();

  }

  public void driveFieldRelative(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
    ChassisSpeeds speeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, pigeon.getRotation2d())
        : ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, pigeon.getRotation2d());
    states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DrivetrainConstants.MAX_VELOCITY);
    for (int i = 0; i <= 3; i++) {
      SwerveModuleState optimizedState = SwerveModuleState.optimize(states[i],
          Rotation2d.fromDegrees(modules[i].getRotFromEncoderPos()));
      modules[i].setTranslation(optimizedState.speedMetersPerSecond);
      modules[i].setRotation(optimizedState.angle.getRotations());
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
    SlewRateLimiter joystickOptimizer = new SlewRateLimiter(1.5);
    double xSpeedOptimized = joystickOptimizer.calculate(xSpeed * Constants.DrivetrainConstants.MAX_VELOCITY);
    double ySpeedOptimized = joystickOptimizer.calculate(ySpeed * Constants.DrivetrainConstants.MAX_VELOCITY);
    double rotOptimized = joystickOptimizer.calculate(rot * Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY);
    ChassisSpeeds controllerSpeeds = new ChassisSpeeds(xSpeedOptimized, ySpeedOptimized, rotOptimized);
    driveFieldRelative(controllerSpeeds, true);
  }

  public void testMotor(double input) {
    modules[0].setTranslation(input);
  }

  public void setMotorsInput(double input) {

    for (Module module : modules) {
      module.setInputToMotor(input);
    }
  }

  // StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
  //     .getStructArrayTopic("My states", SwerveModuleState.struct).publish();
  @Override
  public void periodic() {
    // publisher.set(getStates());

    double[] loggingState = {4,3,5,6,7,8,9,2};
    SmartDashboard.putNumberArray("Swerve state", loggingState);
    
  }

  public static void main(String[] args) {

  }
}

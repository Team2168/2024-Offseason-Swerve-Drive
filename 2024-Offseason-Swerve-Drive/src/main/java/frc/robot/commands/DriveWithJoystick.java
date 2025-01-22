// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.OI;
import frc.robot.subsystems.SwerveDrive;

public class DriveWithJoystick extends Command {
 SwerveDrive swerveDrive;
 OI oi;
 SlewRateLimiter xLimiter;
 SlewRateLimiter yLimiter;
 SlewRateLimiter thetaLimiter;
  public DriveWithJoystick(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;

    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oi = OI.getInstance();
    xLimiter = new SlewRateLimiter(7);
    yLimiter = new SlewRateLimiter(7);
    thetaLimiter = new SlewRateLimiter(5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xNormalized = xLimiter.calculate(-oi.getRightY());
    double yNormalized = yLimiter.calculate(-oi.getRightX());
    double thetaNormalized = thetaLimiter.calculate(-oi.getLeftX());
    swerveDrive.driveController(xNormalized, yNormalized, thetaNormalized);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.driveFieldRelative(new ChassisSpeeds(0,0,0),false,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

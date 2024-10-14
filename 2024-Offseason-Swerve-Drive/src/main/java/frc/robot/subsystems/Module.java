// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Module extends SubsystemBase {
  private TalonFX driveMotor;
  private TalonFX azimuthMotor;
  private CANcoder azimuthEncoder;
  private TalonFXConfiguration driveConfiguration;
  private TalonFXConfiguration azimuthConfiguration;
  private InvertedValue invert = InvertedValue.Clockwise_Positive;
  private NeutralModeValue brake = NeutralModeValue.Brake;
  private MotorOutputConfigs driveMotorOutput;
  private MotorOutputConfigs azimuthMotorOutput;
  private Slot0Configs driveSlot0Configs;
  private Slot0Configs azimuthSlot0Configs;
  private MotionMagicConfigs azimuthMagicConfigs;
  private MotionMagicVoltage azimuthVoltage;
  private VelocityVoltage driveMotorVelocity;
  private VoltageConfigs driveMotorVelocityConfigs;
  private FeedbackConfigs azimuthFeedback;
  private DutyCycleOut driveCycleOut = new DutyCycleOut(0.0);
  private double deadband = 0.002;
  private double drivekP = 0.05;
  private double drivekI = 0.0025;
  private double drivekD = 0.005;
  private double azimuthkP = 0.05;
  private double azimuthkI = 0.0025;
  private double azimuthkD = 0.005;

  // TODO
  // pid, connect cancoder to configs, motoroutputconfigs, motionmagic, feedback.

  public Module(TalonFX driveMotor, TalonFX azimuthMotor, CANcoder azimuthEncoder) {
    this.driveMotor = driveMotor;
    this.azimuthMotor = azimuthMotor;
    this.azimuthEncoder = azimuthEncoder;

    driveConfiguration = new TalonFXConfiguration();
    azimuthConfiguration = new TalonFXConfiguration();

    azimuthVoltage = new MotionMagicVoltage(0.0);
    azimuthMagicConfigs = azimuthConfiguration.MotionMagic;
    azimuthMagicConfigs.withMotionMagicAcceleration(0.0025);
    azimuthMagicConfigs.withMotionMagicCruiseVelocity(0.01);

    driveMotorVelocity = new VelocityVoltage(0.0);
    driveMotorVelocityConfigs = driveConfiguration.Voltage;
    driveMotorVelocityConfigs.withPeakForwardVoltage(10);
    driveMotorVelocityConfigs.withPeakReverseVoltage(10);

    driveMotorOutput = driveConfiguration.MotorOutput;
    azimuthMotorOutput = azimuthConfiguration.MotorOutput;
    driveSlot0Configs = driveConfiguration.Slot0;
    azimuthSlot0Configs = azimuthConfiguration.Slot0;
    azimuthMagicConfigs = azimuthConfiguration.MotionMagic;
    azimuthFeedback = azimuthConfiguration.Feedback;

    driveMotorOutput.withInverted(invert);
    driveMotorOutput.withNeutralMode(brake);
    driveMotorOutput.withDutyCycleNeutralDeadband(deadband);
    azimuthMotorOutput.withInverted(invert);
    azimuthMotorOutput.withNeutralMode(brake);
    azimuthMotorOutput.withDutyCycleNeutralDeadband(deadband);

    driveSlot0Configs.withKP(drivekP);
    driveSlot0Configs.withKI(drivekI);
    driveSlot0Configs.withKD(drivekD);
    azimuthSlot0Configs.withKP(azimuthkP);
    azimuthSlot0Configs.withKI(azimuthkI);
    azimuthSlot0Configs.withKD(azimuthkD);

    azimuthFeedback.withRemoteCANcoder(azimuthEncoder);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

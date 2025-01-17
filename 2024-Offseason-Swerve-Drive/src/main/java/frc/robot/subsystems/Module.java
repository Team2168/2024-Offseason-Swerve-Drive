// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
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
  private MotionMagicConfigs driveMagicConfigs;
  private MotionMagicVoltage driveMotionVoltage;
  private VelocityVoltage driveMotorVelocity;
  private VoltageConfigs driveMotorVelocityConfigs;
  private FeedbackConfigs azimuthFeedback;
  private CurrentLimitsConfigs driveCurrent;
  private CurrentLimitsConfigs azimuthCurrent;
  private DutyCycleOut driveCycleOut = new DutyCycleOut(0.0);
  private double deadband = 0.002;
  private double drivekP = 1;
  private double drivekI = 0.0025;
  private double drivekD = 0.005;
  private double driveKs = 10;
  private double driveKv = 5;
  private double driveKa = 5;
  private double azimuthkP = 1;
  private double azimuthkI = 0.0025;
  private double azimuthkD = 0.005;
  private double motionAccel = 120;
  private double cruiseVelocity = 100;
  private MotionMagicVelocityVoltage volts;

  private double wheelDiameter = Units.inchesToMeters(1.5);
  private double driveGearRatio;
  private double azimuthGearRatio;
  private double wheelCircumference = Math.PI * wheelDiameter;

  // TODO
  // pid, connect cancoder to configs, motoroutputconfigs, motionmagic, feedback.
  // ks, kv and ka for drive (velocity) motors, ks and kv for velocity, use
  // phoenix or sysid.

  public Module(int driveID, int azimuthID, int canCoderID, double steeringGearRation, double driveGearRatio) {
    this.driveGearRatio = driveGearRatio;
    this.azimuthGearRatio = steeringGearRation;
    driveMotor = new TalonFX(driveID, "RoboRIO");
    azimuthMotor = new TalonFX(azimuthID);
    azimuthEncoder = new CANcoder(canCoderID);
    driveConfiguration = new TalonFXConfiguration();
    azimuthConfiguration = new TalonFXConfiguration();
    azimuthVoltage = new MotionMagicVoltage(0.0);
    driveMotionVoltage = new MotionMagicVoltage(0.0);
    azimuthMagicConfigs = azimuthConfiguration.MotionMagic;
    driveMagicConfigs = driveConfiguration.MotionMagic;
    azimuthMagicConfigs.withMotionMagicAcceleration(motionAccel);
    azimuthMagicConfigs.withMotionMagicCruiseVelocity(cruiseVelocity);
    driveMagicConfigs.withMotionMagicAcceleration(motionAccel);
    driveMagicConfigs.withMotionMagicCruiseVelocity(cruiseVelocity);

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
    driveCurrent = driveConfiguration.CurrentLimits;
    azimuthCurrent = azimuthConfiguration.CurrentLimits;

    driveMotorOutput.withInverted(invert);
    driveMotorOutput.withNeutralMode(brake);
    driveMotorOutput.withDutyCycleNeutralDeadband(deadband);
    
    azimuthMotorOutput.withInverted(invert);
    azimuthMotorOutput.withNeutralMode(brake);
    azimuthMotorOutput.withDutyCycleNeutralDeadband(deadband);

    driveSlot0Configs.withKP(drivekP);
    driveSlot0Configs.withKI(drivekI);
    driveSlot0Configs.withKD(drivekD);
    driveSlot0Configs.withKS(driveKs);
    azimuthSlot0Configs.withKP(azimuthkP);
    azimuthSlot0Configs.withKI(azimuthkI);
    azimuthSlot0Configs.withKD(azimuthkD);

    driveCurrent.withSupplyCurrentLimitEnable(true);
    azimuthCurrent.withSupplyCurrentLimitEnable(true);
    driveCurrent.withSupplyCurrentLimit(35);
    driveCurrent.withSupplyCurrentThreshold(40);
    driveCurrent.withSupplyTimeThreshold(0.1);
    azimuthCurrent.withSupplyCurrentLimit(10);
    azimuthCurrent.withSupplyCurrentThreshold(15);
    azimuthCurrent.withSupplyTimeThreshold(0.1);

    azimuthFeedback.withRemoteCANcoder(azimuthEncoder);

    driveMotor.getConfigurator().apply(driveConfiguration);
    azimuthMotor.getConfigurator().apply(azimuthConfiguration);

  }

  public void setTranslation(double meters) {
    driveMotor.setControl(driveMotorVelocity.withVelocity(metersToRot(meters)));
  }

  public void setRotation(double rotations) {
    azimuthMotor.setControl(azimuthVoltage.withPosition((rotations * azimuthGearRatio)));
  }

  public void setInput(double meters, double rotations) {
    setTranslation(meters);
    setRotation(rotations);
  }

  public void setInputToMotor(double input) {
    DutyCycleOut output = new DutyCycleOut(input);
    driveMotor.setControl(output);
    azimuthMotor.setControl(output);
  }

  public void dutyCycle() {
    return;
  }

  public void resetEncoderToAbsolute() {
    return;
  }

  public double getRotation() {
    return 0.0;
  }

  public double getTranslationMeters() {
    return 0.0;
  }

  public double rotToMeters(double rotations) {
    double metersTraveled = (rotations / driveGearRatio) * wheelCircumference;
    return metersTraveled;
  }

  public double rpmToVelocity(double rpm) {
    double velocity = rotToMeters(rpm) / 60;
    return velocity;
  }

  public double metersToRot(double meters) {
    double rot = (meters * driveGearRatio) / wheelCircumference;
    return rot;
  }

  // public double metersToSensorUnits(double rot) {
  // return metersToRot(rot)/60;
  // }

  public double getRotFromEncoderPos() {
    return Units.rotationsToDegrees(azimuthMotor.getPosition().getValue()) / azimuthGearRatio;
  }

  public double getAngleDegrees() {
    return Units.rotationsToDegrees(azimuthMotor.getPosition().getValue() / azimuthGearRatio);
  }

  public void resetEncoder() {
    driveMotor.setPosition(0.0);
    azimuthMotor.setPosition(0.0);
  }



  // public double

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

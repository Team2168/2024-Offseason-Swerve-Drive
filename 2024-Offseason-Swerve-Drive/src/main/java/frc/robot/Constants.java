// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static int kDriverControllerPort = 0;
  }

  public static class DrivetrainConstants {
    public static final int FL_DRIVE = 9;//mk4 steering: 12.8:1; driving: 8.14:1
    public static final int FR_DRIVE = 10; //mk4 steering: 12.8:1; driving: 8.14:1
    public static final int BL_DRIVE = 11; //wcpx steering: 13.3714:1, driving: 8.10:1
    public static final int BR_DRIVE = 12; //wcpx steering: 13.3714:1, driving: 8.10:1
    public static final int FL_AZIMUTH = 1; // change
    public static final int FR_AZIMUTH = 2; // change
    public static final int BL_AZIMUTH = 3; // change
    public static final int BR_AZIMUTH = 4; // change must check phoenix 6
    public static final int FL_CANCODER_ID = 5;
    public static final int FR_CANCODER_ID = 6;
    public static final int BL_CANCODER_ID = 7;
    public static final int BR_CANCODER_ID = 8;

    public static final int PIGEON_ID = 17; //change

    public static double Y_POS_MODULE = Units.inchesToMeters(10);
    public static double X_POS_MODULE = Units.inchesToMeters(12);
    
    public static final double MAX_VELOCITY = 4.00; //m/s
    public static final double MAX_ANGULAR_VELOCITY = 3.00;

    public static int[] driveIDs = {FL_DRIVE, FR_DRIVE, BL_DRIVE, BR_DRIVE};
    public static int[] azimuthIDs = {FL_AZIMUTH, FR_AZIMUTH, BL_AZIMUTH, BR_AZIMUTH};
    public static int[] cancoderIDs = {FL_CANCODER_ID, FR_CANCODER_ID, BL_CANCODER_ID, BR_CANCODER_ID};
    public static double[] drivingRatios = {8.14, 8.14, 8.10, 8.10};
    public static double[] steeringRatios = {12.8, 12.8, 13.3714, 13.3714};
  }

  public static class MotorConstants {
    public static final int SIMULATED_MOTOR_ID = 14;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  }

  public static class DrivetrainConstants {
    public static final int FL_DRIVE = 3;
    public static final int FR_DRIVE = 2;
    public static final int BL_DRIVE = 1;
    public static final int BR_DRIVE = 0;
    public static final int FL_AZIMUTH = 3; // change
    public static final int FR_AZIMUTH = 2; // change
    public static final int BL_AZIMUTH = 1; // change
    public static final int BR_AZIMUTH = 0; // change
    public static final int FL_CANCODER_ID = 3;
    public static final int FR_CANCODER_ID = 2;
    public static final int BL_CANCODER_ID = 1;
    public static final int BR_CANCODER_ID = 0;

    public static final int PIGEON_ID = 0;

    public static final double FL_X = -0.381; // meters
    public static final double FL_Y = 0.381;
    public static final double FR_X = 0.381;
    public static final double FR_Y = 0.381;
    public static final double BL_X = -0.381; // meters
    public static final double BL_Y = -0.381;
    public static final double BR_X = 0.381;
    public static final double BR_Y = -0.381;
    
    public static final double MAX_VELOCITY = 4.00; //m/s
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double FIELD_WIDTH = 16.54; // in meters
  public static final double FIELD_HEIGHT = 8.02; // in meters

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ArmConstants{
    public static final int TICK_PER_DEGREE = 250;
    public static final double SECTION_1_KP = 1;
    public static final double SECTION_1_KI = 0;
    public static final double SECTION_1_KD = 0;
    public static final double SECTION_2_KP = 1;
    public static final double SECTION_2_KI = 0;
    public static final double SECTION_2_KD = 0;
    public static final double SECTION_1_PID_TOLERANCE = 0.1;
    public static final double SECTION_2_PID_TOLERANCE = 0.1;
    public static final double SECTION_1_MAX_ACCELERATION = 90;
    public static final double SECTION_2_MAX_ACCELERATION = 90;
    public static final double SECTION_1_LENGTH = 1;
    public static final double SECTION_2_LENGTH = 1;
    public static final double DISPLAY_ARM_MULTIPLIER = 4;
    public static final double MAX_LENGTH = SECTION_1_LENGTH + SECTION_2_LENGTH - 0.0001;
   
  }

}

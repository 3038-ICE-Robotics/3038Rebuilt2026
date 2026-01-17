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
    public static final int kDriverControllerPort = 0;
  }

  public static class DigitalChannels {
    public static final int HopperFull = 0;
  }

  public static class MotorSpeeds {
    public static final double IntakeSpeed = 0.75;
  }

  public static class ButtonIDs {
    public static final int Intake = 0;
    public static final int Outtake = 1;
  }

  public static class MotorIDs {
    public static final int[] DriveIDs = new int[] { 0, 1, 2, 3 };
    public static final int[] SteerIDs = new int[] { 4, 5, 6, 7 };
    public static final int ShooterPrime = 8;
    public static final int ShooterFollow = 9;
    public static final int Transfer = 10;
    public static final int IntakePrime = 11;
    public static final int IntakeFollow = 12;
    public static final int ClimbPrime = 13;
    public static final int ClimbFollow = 14;
  }
  public static class NeoVortex {
    public static final int StallCurrent = 211;
    public static final double CurrentThreshhold = 40;
   }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

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
  public static final double MetersToFeet = 3.28084;

  public static class OperatorConstants {
    public static final int LDriverControllerPort = 0;
    public static final int RDriverControllerPort = 1;
  }

  public static class DigitalChannels {
    public static final int HopperFull = 0;
    public static final int HopperEmpty = 1;
  }

  public static class MotorSpeeds {
    public static final double IntakeSpeed = 0.75;
  }

  public static class LeftButtonIDs {
    public static final int Intake = 10;
    public static final int Outtake = 9;
    public static final int IntakeToHopper = 1;
    public static final int OutTakeFull = 2;
    public static final int ExtendClimber = 7;
    public static final int RetractClimber = 6;
  }

  public static class RightButtonIDs {
    public static final int ShootFromHopper = 1;
    public static final int ShootFromIntake = 2;

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

  public static class DriveTrain {
    public static final double WheelDiameter = (4 / 12) / MetersToFeet;
    public static final double DrivetrainTrackWidth = (20.5 / 12.0) / MetersToFeet;
    public static final double DrivetrainWheelbase = (20.5 / 12.0) / MetersToFeet;
    public static final double RotationsToMeters = Math.PI * WheelDiameter;
    public static final double MaxVelocityRPSEmpirical = (10 * MetersToFeet) / RotationsToMeters;
    public static final double RotationkP = 0;
    public static final double RotationkI = 0;
    public static final double RotationkD = 0;
    public static final double RotationTolerance = 0;
    public static final String[] MotorKeys = new String[] { "FL", "FR", "BL", "BR" };
    public static final Pose2d DriveOdometryOrigin = new Pose2d(5, 5, new Rotation2d());
    public static final double MaxVelocityMPS = 5; // measure actual velocity
    public static final double MaxAngularVelocityRadiansPS = MaxVelocityMPS
        / Math.hypot(DrivetrainTrackWidth / 2.0, DrivetrainWheelbase / 2.0);; // measure
    public static final double DriveDeadbandMPS = 1; // measure
    public static final PIDController DRIVE_TO_POSE_X_CONTROLLER = getTranslationPIDController();
    public static final PIDController DRIVE_TO_POSE_Y_CONTROLLER = getTranslationPIDController();

    public static PIDController getTranslationPIDController() {
      PIDController transltionController = new PIDController(5, 0, 0);
      transltionController.setIZone(0.025);
      transltionController.setIntegratorRange(-0.25, 0.25);
      return transltionController;
    }
  }

  public static class Limelight {
    public static final String LimelightName = "LimelightA";
    public static final Translation2d FieldCorner = new Translation2d(17.54, 8.02);
    public static final double MaxTagDistance = 3;
  }

  public static class Field {
    public static final Translation2d BlueHub = new Translation2d(4.6, 4);
    public static final Translation2d RedHub = new Translation2d(11.9, 4);
  }

  public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(null);
}

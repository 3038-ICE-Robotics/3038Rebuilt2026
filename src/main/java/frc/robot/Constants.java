// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/*
 * To access the Rio logs
 * open command prompt and use:
 * ssh lvuser@roboRIO-3038-frc.local
 * cd logs
 * ls
 * that will show how many log files there are
 * then use the following to remove the files
 * rm *.revlog
 */

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
    public static final int LDriverControllerPort = 2;
    public static final int RDriverControllerPort = 3;
  }

  public static class DigitalChannels {
    public static final int HopperFull = 2;
    public static final int HopperEmpty = 4;
    public static final int RightClimbHome = 0;
    public static final int LeftClimbHome = 1;
  }

  public static class Climb {
    public static final double ExtendSpeed = 0.4;
    public static final double RetractSpeed = 0.15;
    public static final double ExtendHeight = 90;
  }

  public static class AimBotData {
    public static double[] RotationPID = new double[] { // TODO: adjust this to control how aggressive the aimbot turn is
        0.1, // kP
        0, // kI
        0 // kD
    };
    public static double[] distancesToHub = new double[] {
        40.125, // closest possible distance
        80,
        120,
        160,
        200,
        240, 
    };
    public static double[] shooterSpeeds = new double[] { // TODO measure (RPM)
        1350,
        2710,
        3060,
        3450 + 200,
        3850 + 200,
        4300 + 200
    };
  }

  public static class MotorSpeeds {
    public static final double IntakeSpeed = 0.28;
    public static final double RearIntakeSpeed = -0.60;
    public static final double RearSpeed = 0.28;
    
  }

  public static class HippoData {
    public static final double RetractLimitL = 0.750;
    public static final double RetractLimitR = 0.564;
    public static final double ExtendLimitL = 0.493;
    public static final double ExtendLimitR = 0.314;
    public static final double AgitateLimitL = 0.641;
    public static final double AgitateLimitR = 0.477;
  }

  public static class LeftButtonIDs {
    public static final int Intake = 10;
    public static final int IntakeToHopper = 1;
    public static final int ToggleAimBot = 3;
    public static final int RearIntake = 1;
    public static final int RearExtend = 4;
    public static final int RearRetract = 5;
  }

  public static class LEDs {
    public static final int CHANNEL_2_PIN = 2;
    public static final int CHANNEL_3_PIN = 3;
    public static final int CHANNEL_4_PIN = 4;
  }

  public static class RightButtonIDs {
    public static final int ShootFromHopper = 1;
    public static final int ShootFromIntake = 2;
    public static final int OutTakeToGround = 3;
    public static final int RightClimbRetract = 4;
    public static final int RightClimbExtend = 5;

  }

  public static class MotorIDs {
    // Order of drive motors - FL, FR, BL, BR
    public static final int[] DriveIDs = new int[] { 8, 1 , 2, 3 }; 
    public static final int[] SteerIDs = new int[] { 4, 5, 6, 7 };

    public static final int ShooterPrime = 15;
    public static final int ShooterFollow = 9;
    public static final int Intake = 10;
    public static final int TransferLow = 11;
    public static final int TransferHigh = 12;
    public static final int ClimbRight = 13;
    public static final int ClimbLeft = 14;
    public static final int RearLeft = 24;
    public static final int RearRight = 25;
    public static final int RearIntake = 26;
  }

  public static class NeoVortex {
    public static final int StallCurrent = 211;
    public static final double CurrentThreshhold = 40;
  }

  public static class DriveTrain {
    public static final double SteerGearRatio = (94.0 / 18.0) * 2.89 * 3.61;
    public static final double DriveGearRatio = 7.5;
    public static final double[] ModuleOffsets = new double[] {
        -0.269,
        -0.438,
        -0.766,
        -0.959
    };
    public static final double WheelDiameter = (4.0 / 12.0) / MetersToFeet;
    public static final double DrivetrainTrackWidth = (20.5 / 12.0) / MetersToFeet;
    public static final double DrivetrainWheelbase = (20.5 / 12.0) / MetersToFeet;
    public static final double RotationsToMeters = (Math.PI * WheelDiameter) / DriveGearRatio;
    public static final double MaxVelocityRPSEmpirical = (10 * MetersToFeet) / RotationsToMeters;
    //TODO: adjust these to make pathplanner correct for heading errors.
    public static final double RotationkP = 0;
    public static final double RotationkI = 0;
    public static final double RotationkD = 0;
    //TODO: adjust these to make pathplanner correct for position errors
    public static final double TranslationkP = 0;
    public static final double TranslationkI = 0;
    public static final double TranslationkD = 0;
    public static final double RotationTolerance = 0;
    public static final String[] MotorKeys = new String[] { "FL", "FR", "BL", "BR" };
    public static Pose2d DriveOdometryOrigin = new Pose2d();
    //TODO: the theoretical max is closer to 8 or 9
    // measuring the actual max means pointing the wheels forward and just sending full power to the drive wheels and measuring the top speed reached.
    public static final double MaxVelocityMPS = 5;
    //TODO: This is is just some guestimate math and should be updated similar to MaxVelocityMPS
    // measuring the actual max means point the wheels in a rotating orientation and then sending full power to the wheels and measure the top Radians per second reached.
    public static final double MaxAngularVelocityRadiansPS = MaxVelocityMPS
        / Math.hypot(DrivetrainTrackWidth / 2.0, DrivetrainWheelbase / 2.0); 
    public static final double DriveDeadbandMPS = 0.1;
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
    public static final String LimelightTwoName = "limelight-two";
    public static final String LimelightOneName = "limelight-one";
    public static final Translation2d FieldCorner = new Translation2d(17, 8.02);
    public static final double MaxTagDistance = 10;
  }

  public static class Field {
    public static final Translation2d BlueHub = new Translation2d(4.626, 4.035); // meters \/
    public static final Translation2d RedHub = new Translation2d(11.915, 4.035);
    public static final Translation2d BlueZone = new Translation2d(.5, 0);
    public static final Translation2d RedZone = new Translation2d(16.5, 0);
    //TODO: update to meters and 
    public static final double FieldLength = 650.12; // inches \/
    public static final double FieldWidth = 316.64;
    public static final double BlueBoundary = 180;
    public static final double RedBoundary = FieldLength - BlueBoundary;

  }

  public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(DriveTrain.DrivetrainTrackWidth / 2, DriveTrain.DrivetrainWheelbase / 2),
      new Translation2d(DriveTrain.DrivetrainTrackWidth / 2, DriveTrain.DrivetrainWheelbase / -2),
      new Translation2d(DriveTrain.DrivetrainTrackWidth / -2, DriveTrain.DrivetrainWheelbase / 2),
      new Translation2d(DriveTrain.DrivetrainTrackWidth / -2, DriveTrain.DrivetrainWheelbase / -2));
}

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class Drive extends Command {

  private final DriveSubsystem drivetrain;
  private final BooleanSupplier robotCentricMode;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;
  private final DoubleSupplier speedSupplier;
  private int invert=1;

  /**
   * drives the robot at a specific forward velocity, sideways velocity, and
   * rotational velocity.
   *
   * @param drivetrainSubsystem  Swerve drive subsytem
   * @param robotCentricMode     while this is pressed, the robot will drive in
   *                             RobotCentric mode.
   *                             Otherwise, it will default to field centric
   * @param translationXSupplier forward throttle (from -1 to 1). 1 will drive at
   *                             full speed forward
   * @param translationYSupplier sideways throttle (from -1 to 1). 1 will drive at
   *                             full speed to the
   *                             right
   * @param rotationSupplier     rotational throttle (from -1 to 1). 1 will drive
   *                             at full speed
   *                             clockwise
   */
  public Drive(
      DriveSubsystem drivetrainSubsystem,
      BooleanSupplier robotCentricMode,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier speedSupplier,
      DoubleSupplier rotationSupplier) {
    this.drivetrain = drivetrainSubsystem;
    this.robotCentricMode = robotCentricMode;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.speedSupplier = speedSupplier;
    this.rotationSupplier = rotationSupplier;
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void execute() {
    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
    // field-oriented
    // movement
    if (DriverStation.getAlliance().get() == Alliance.Red) {
     invert = -1;
   } else {
     invert = 1;
   }
    // The two statements are mostly identical, taking X, Y, and Rotation suppliers
    // and multiplying them by maximum velocties and inversions
    // The only difference is that one is relative to the field, and the other to
    // the robot.
    double distancefromBoundary = Math.abs(drivetrain.getPose().getX() - Constants.Field.BlueBoundary);
    double boundarySpeedModifier = (distancefromBoundary <= 40) ? 0.5 : 1;
    double modifiedSpeed = speedSupplier.getAsDouble();//Constants.DriveTrain.MaxVelocityMPS * boundarySpeedModifier;
    if (robotCentricMode.getAsBoolean()) {
      drivetrain.drive(
          new ChassisSpeeds(
              translationXSupplier.getAsDouble()
                  * (modifiedSpeed),
              translationYSupplier.getAsDouble()
                  * (modifiedSpeed),
              rotationSupplier.getAsDouble()
                  * Constants.DriveTrain.MaxAngularVelocityRadiansPS));
    } else {
      drivetrain.drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translationXSupplier.getAsDouble()
                  * (modifiedSpeed)
                  * invert,
              translationYSupplier.getAsDouble()
                  * (modifiedSpeed)
                  * invert,
              rotationSupplier.getAsDouble()
                  * Constants.DriveTrain.MaxAngularVelocityRadiansPS,
              drivetrain.getPose().getRotation()));
    }

    SmartDashboard.putBoolean("Inputs/Robot Centric", robotCentricMode.getAsBoolean());
    SmartDashboard.putNumber("Inputs/x", translationXSupplier.getAsDouble());
    SmartDashboard.putNumber("Inputs/y", translationYSupplier.getAsDouble());
    SmartDashboard.putNumber("Inputs/z", rotationSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}

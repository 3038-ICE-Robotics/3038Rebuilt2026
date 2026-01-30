// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//docs.advantagekit.org/data-flow/recording-inputs/io-interfaces/
package frc.robot;

import frc.robot.Constants.DriveTrain;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Drive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SystemCommands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import edu.wpi.first.wpilibj.Joystick;

import java.io.IOException;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private DriveSubsystem drivetrain;
  private Drive defaultDriveCommand;
  DoubleSupplier ControllerForwardAxisSupplier;
  DoubleSupplier ControllerSidewaysAxisSupplier;
  DoubleSupplier ControllerZAxisSupplier;
  Joystick lJoystick;
  Joystick rJoystick;
  SystemCommands fullCommands;

  private void driveTrainInit() {
    drivetrain = new DriveSubsystem();

    defaultDriveCommand = new Drive(
        drivetrain,
        () -> false,
        ControllerForwardAxisSupplier,
        ControllerSidewaysAxisSupplier,
        ControllerZAxisSupplier);
    drivetrain.setDefaultCommand(defaultDriveCommand);
  }

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private CommandJoystick commandJoystickL;
  private CommandJoystick commandJoystickR;
  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;
  private TransferSubsystem transfer;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    lJoystick = new Joystick(Constants.OperatorConstants.LDriverControllerPort);
    rJoystick = new Joystick(Constants.OperatorConstants.RDriverControllerPort);
    // Drive controls
    ControllerSidewaysAxisSupplier = () -> modifyAxis(-lJoystick.getX(), 0);
    ControllerForwardAxisSupplier = () -> modifyAxis(-lJoystick.getY(), 0);
    ControllerZAxisSupplier = () -> modifyAxis(-rJoystick.getX(), 0);
    // set stuff
    commandJoystickL = new CommandJoystick(Constants.OperatorConstants.LDriverControllerPort);
    intake = new IntakeSubsystem();
    shooter = new ShooterSubsystem();
    transfer = new TransferSubsystem();
    fullCommands = new SystemCommands(intake, transfer, shooter);
    // Configure the trigger bindings
    configureBindings();
    driveTrainInit();
    // configureDriveTrain();
  }

  private double modifyAxis(double value, double deadband) {
    // Deadband
    value = MathUtil.applyDeadband(value, deadband);
    // Square the axis
    value = Math.copySign(value * value, value);
    return value;
  }
  // Pathplanner TODO
  // private void configureDriveTrain() {
  // try {
  // AutoBuilder.configure(
  // drivetrain::getPose, // Pose2d supplier
  // drivetrain::resetOdometry, // Pose2d consumer, used to reset odometry at the
  // beginning of auto
  // drivetrain::getChassisSpeeds,
  // (speeds) -> drivetrain.drive(speeds),
  // new PPHolonomicDriveController(
  // new com.pathplanner.lib.config.PIDConstants(
  // k_XY_P, k_XY_I,
  // k_XY_D), // PID constants to correct for translation error (used to create
  // the X
  // // and Y PID controllers)
  // new com.pathplanner.lib.config.PIDConstants(
  // k_THETA_P, k_THETA_I,
  // k_THETA_D) // PID constants to correct for rotation error (used to create the
  // // rotation controller)
  // ),
  // RobotConfig.fromGUISettings(),
  // () -> DriverStation.getAlliance().get().equals(Alliance.Red),
  // drivetrain);
  // } catch (org.json.simple.parser.ParseException a) {
  // System.out.println("got ParseException trying to configure AutoBuilder");
  // } catch (IOException b) {
  // System.out.println("got IOException thrown trying to configure autobuilder "
  // + b.getMessage());
  // }
  // }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
    // // Run intake while holding the intake button, and stop intake on release
    // commandJoystickL.button(Constants.LeftButtonIDs.Intake)
    //     .onTrue(new InstantCommand(intake::startIntake))
    //     .and(() -> !commandJoystickL.getHID().getRawButton(Constants.LeftButtonIDs.Outtake))
    //     .onFalse(new InstantCommand(intake::stop));
    // // Run outtake while holding the outtake button, and stop outtake on release
    // commandJoystickL.button(Constants.LeftButtonIDs.Outtake)
    //     .onTrue(new InstantCommand(intake::startOuttake))
    //     .and(() -> !commandJoystickL.getHID().getRawButton(Constants.LeftButtonIDs.Intake))
    //     .onFalse(new InstantCommand(intake::stop));
    commandJoystickL.button(Constants.LeftButtonIDs.IntakeToHopper)
        .onTrue(fullCommands.intakeBall)
        .onFalse(new InstantCommand(fullCommands.intakeBall::cancel));
    commandJoystickL.button(Constants.LeftButtonIDs.OutTakeFull)
        .onTrue(fullCommands.outtakeBall)
        .onFalse(new InstantCommand(fullCommands.outtakeBall::cancel));
    commandJoystickR.button(Constants.RightButtonIDs.ShootFromHopper)
        .onTrue(fullCommands.shootBallFromHopper)
        .onFalse(new InstantCommand(fullCommands.shootBallFromHopper::cancel));
    commandJoystickR.button(Constants.RightButtonIDs.ShootFromIntake)
        .onTrue(fullCommands.shootBallFromGround)
        .onFalse(new InstantCommand(fullCommands.shootBallFromGround::cancel));
    // .onTrue(new IntakeCommand(intake))
    // .onFalse(new StopIntakeCommand(intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}

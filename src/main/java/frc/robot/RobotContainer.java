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
import frc.robot.interfaces.ITunable;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.RearSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import edu.wpi.first.wpilibj.Joystick;

import java.io.IOException;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

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

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private CommandJoystick commandJoystickL;
  private CommandJoystick commandJoystickR;
  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;
  private TransferSubsystem transfer;

  private ClimberSubsystem climb;
  private LedSubsystem led;
  private RearSubsystem rear;
  private SysIdRoutine sysRoutine1;
  private Config configForSysRoutine1;
  private SendableChooser<ITunable> subSystemChooser = new SendableChooser<ITunable>();
  private SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      Constants.DriveTrain.DriveOdometryOrigin = new Pose2d(16, 7.5, new Rotation2d());
    } else {
      Constants.DriveTrain.DriveOdometryOrigin = new Pose2d(0, 0, new Rotation2d());
    }

    lJoystick = new Joystick(Constants.OperatorConstants.LDriverControllerPort);
    rJoystick = new Joystick(Constants.OperatorConstants.RDriverControllerPort);
    // Drive controls
    ControllerSidewaysAxisSupplier = () -> modifyAxis(-lJoystick.getX(), 0.075);
    ControllerForwardAxisSupplier = () -> modifyAxis(-lJoystick.getY(), 0.075);
    ControllerZAxisSupplier = () -> modifyAxis(-rJoystick.getX(), 0.075);
    // set stuff
    commandJoystickL = new CommandJoystick(Constants.OperatorConstants.LDriverControllerPort);
    commandJoystickR = new CommandJoystick(Constants.OperatorConstants.RDriverControllerPort);
    intake = new IntakeSubsystem();
    transfer = new TransferSubsystem();
    climb = new ClimberSubsystem();
    // led = LedSubsystem.getInstance();
    rear = new RearSubsystem();
    driveTrainInit();
    shooter = new ShooterSubsystem(drivetrain::getPose);
    shooter.speedControl = () -> {
      return ((rJoystick.getZ() + 1) / 2) * 3500 + 1500;
    };
    shooter.distanceControl = () -> {
      return ((lJoystick.getZ() + 1) / 2) * 185 + 40;
    };
    fullCommands = new SystemCommands(intake, transfer, shooter, rear, drivetrain);

    // Configure Auto's
    // configureNamedCommands();
    NamedCommands.registerCommand("shoot", fullCommands.shootBallFromHopper);
    NamedCommands.registerCommand("intake", fullCommands.intakeBall);
    NamedCommands.registerCommand("flip out", fullCommands.rearExtend);
    NamedCommands.registerCommand("flip in", fullCommands.rearRetract);
    NamedCommands.registerCommand("climb extend", climb.extend);
    NamedCommands.registerCommand("climb retract", climb.retract);
    NamedCommands.registerCommand("outtake", fullCommands.outtakeBall);

    configureAutoBuilder();
    // NamedCommands.registerCommand("flip out", new
    // ParallelCommandGroup(rear.extendLeft,rear.extendRight));
    // NamedCommands.registerCommand("intake", fullCommands.intakeBall);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    SmartDashboard.putData("Tuning/set", new InstantCommand(this::updatePID));
    SmartDashboard.putData("Tuning/Selection", subSystemChooser);

    // Configure the trigger bindings
    configureBindings();

  }

  private double modifyAxis(double value, double deadband) {
    // Deadband
    value = MathUtil.applyDeadband(value, deadband);
    // Square the axis
    value = Math.copySign(value * value, value);
    return value;
  }

  private void driveTrainInit() {
    drivetrain = new DriveSubsystem();

    defaultDriveCommand = new Drive(
        drivetrain,
        () -> false,
        ControllerForwardAxisSupplier,
        ControllerSidewaysAxisSupplier,

        () -> StateOfRobot.isAimAssistOn
            ? StateOfRobot.getAimBotRotation(drivetrain::getDesiredRobotAngle, () -> drivetrain.getPose().getRotation())
            : ControllerZAxisSupplier.getAsDouble());
    drivetrain.setDefaultCommand(defaultDriveCommand);

  }

  // Pathplanner
  public void configureAutoBuilder() {
    try {
      AutoBuilder.configure(
          drivetrain::getPose, // Pose2d supplier
          drivetrain::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
          drivetrain::getChassisSpeeds,
          (speeds) -> {
            // TODO: remove this speed adjustment
            drivetrain.drive(speeds);
          },
          new PPHolonomicDriveController(
              new com.pathplanner.lib.config.PIDConstants(
                  Constants.DriveTrain.TranslationkP,
                  Constants.DriveTrain.TranslationkI,
                  Constants.DriveTrain.TranslationkD), // PID constants to correct for translation error (used to create
                                                       // the X
              // and Y PID controllers)
              new com.pathplanner.lib.config.PIDConstants(
                  Constants.DriveTrain.RotationkP, Constants.DriveTrain.RotationkI,
                  Constants.DriveTrain.RotationkD) // PID constants to correct for rotation error (used to create the
          // rotation controller)
          ),
          RobotConfig.fromGUISettings(),
          () -> DriverStation.getAlliance().get().equals(Alliance.Blue),
          drivetrain);
    } catch (org.json.simple.parser.ParseException a) {
      System.out.println("got ParseException trying to configure AutoBuilder");
    } catch (IOException b) {
      System.out.println("got IOException thrown trying to configure autobuilder " + b.getMessage());
    }
    Autos.loadAutos();
  }

  public void configureNamedCommands() {

  }

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
    // commandJoystickL.button(Constants.LeftButtonIDs.Intake)
    // .onTrue(new InstantCommand(intake::startIntake))
    // .and(() ->
    // !commandJoystickL.getHID().getRawButton(Constants.RightButtonIDs.OuttakeFirstMotor))
    // .onFalse(new InstantCommand(intake::stop));
    // commandJoystickL.button(Constants.RightButtonIDs.OuttakeFirstMotor)
    // .onTrue(new InstantCommand(intake::startOuttake))
    // .and(() ->
    // !commandJoystickL.getHID().getRawButton(Constants.LeftButtonIDs.Intake))
    // .onFalse(new InstantCommand(intake::stop));
    commandJoystickL.button(Constants.LeftButtonIDs.IntakeToHopper)
        .onTrue(fullCommands.intakeBall)
        .onFalse(new InstantCommand(fullCommands.intakeBall::cancel));
    commandJoystickR.button(Constants.RightButtonIDs.OutTakeToGround)
        .onTrue(fullCommands.outtakeBall)
        .onFalse(new InstantCommand(fullCommands.outtakeBall::cancel));
    commandJoystickR.button(Constants.RightButtonIDs.ShootFromHopper)
        .onTrue(fullCommands.shootBallFromHopper)
        .onFalse(new InstantCommand(fullCommands.shootBallFromHopper::cancel));
    commandJoystickR.button(Constants.RightButtonIDs.ShootFromIntake)
        .onTrue(fullCommands.shootBallFromGround)
        .onFalse(new InstantCommand(fullCommands.shootBallFromGround::cancel));
    commandJoystickL.button(Constants.LeftButtonIDs.ToggleAimBot)
        .onTrue(new InstantCommand(StateOfRobot::toggleAimAssist));
    commandJoystickR.button(Constants.RightButtonIDs.RightClimbRetract)
        .onTrue(climb.retract)
        .onFalse(new InstantCommand(climb.retract::cancel));
    commandJoystickR.button(Constants.RightButtonIDs.RightClimbExtend)
        .onTrue(climb.extend)
        .onFalse(new InstantCommand(climb.extend::cancel));
    commandJoystickL.button(Constants.LeftButtonIDs.RearExtend)
        .onTrue(fullCommands.rearExtend)
        .onFalse(new InstantCommand(fullCommands.rearExtend::cancel));
    commandJoystickL.button(Constants.LeftButtonIDs.RearRetract)
        .onTrue(fullCommands.rearRetract)
        .onFalse(new InstantCommand(fullCommands.rearRetract::cancel));
    commandJoystickL.button(Constants.LeftButtonIDs.RearIntake)
        .onTrue(fullCommands.rearIntake)
        .onFalse(new InstantCommand(fullCommands.rearIntake::cancel));
    commandJoystickR.button(Constants.RightButtonIDs.ResetGyro)
        .onTrue(fullCommands.resetGyro);
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
    return autoChooser.getSelected();
  }

  // This function only gets called when the "Tuning/set" button is pressed on
  // Elastic.
  private void updatePID() {

    double kp = SmartDashboard.getNumber("PID/pvalue", Constants.DriveTrain.RotationkP);
    double ki = SmartDashboard.getNumber("PID/ivalue", Constants.DriveTrain.RotationkI);
    double kd = SmartDashboard.getNumber("PID/dvalue", Constants.DriveTrain.RotationkD);

    subSystemChooser.getSelected().updatePID(kp, ki, kd);
    SmartDashboard.putString("Tuning/applied", "true");

  }
}

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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private SysIdRoutine sysRoutine1;
  private Config configForSysRoutine1;
  private SendableChooser<ITunable> subSystemChooser = new SendableChooser<ITunable>();

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
    commandJoystickR = new CommandJoystick(Constants.OperatorConstants.RDriverControllerPort);
    intake = new IntakeSubsystem();
    transfer = new TransferSubsystem();
    driveTrainInit();
    // shooter = new ShooterSubsystem(drivetrain::getPose);
    fullCommands = new SystemCommands(intake, transfer, shooter);
    // Configure the trigger bindings
    configureBindings();

    // configureDriveTrain();
    configForSysRoutine1 = new Config(null, null, null);
    sysRoutine1 = new SysIdRoutine(configForSysRoutine1,
        new SysIdRoutine.Mechanism(drivetrain::voltageDrive, drivetrain::sysLog, m_exampleSubsystem));

    // TODO: move the smartdashboard putnumber calls to this spot.
    // also update the string so that it shows as "Tuning/..." for each of the
    // values.
    // it also might be a good idea to update the default values to 0 instead of a
    // reference to Constants since these will be used for other system tuning.


    // we can add other subsystems to this chooser with addOption(...) and by making
    // each subsystem implement ITunable and adding an override for updatePID
    // function in each subsystem.
    // If you are up for it, try adding this implementation to the shooter subsystem
    // and then adding it to the chooser.
    subSystemChooser.setDefaultOption("Swerve", drivetrain);


    SmartDashboard.putData("Tuning/set",new InstantCommand(this::updatePID));
    SmartDashboard.putData("Tuning/Selection", subSystemChooser);

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
        () -> StateOfRobot.isAimAssistOn ? StateOfRobot.getAimBotRotation(drivetrain.getPose())
            : ControllerZAxisSupplier.getAsDouble());
    drivetrain.setDefaultCommand(defaultDriveCommand);

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
    commandJoystickL.button(Constants.LeftButtonIDs.Intake)
        .onTrue(new InstantCommand(intake::startIntake))
        .and(() -> !commandJoystickL.getHID().getRawButton(Constants.LeftButtonIDs.Outtake))
        .onFalse(new InstantCommand(intake::stop));
    commandJoystickL.button(Constants.LeftButtonIDs.Outtake)
        .onTrue(new InstantCommand(intake::startOuttake))
        .and(() -> !commandJoystickL.getHID().getRawButton(Constants.LeftButtonIDs.Intake))
        .onFalse(new InstantCommand(intake::stop));
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
    commandJoystickL.button(Constants.LeftButtonIDs.ToggleAimBot)
        .onTrue(new InstantCommand(StateOfRobot::toggleAimAssist));
    commandJoystickL.button(11)
        .onTrue(new InstantCommand(drivetrain::setEncoderOffsets));
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


  //This function only gets called when the "Tuning/set" button is pressed on Elastic.
  private void updatePID() {
    if (DriverStation.isTest()) {
      // TODO: move the getnumber pid calls to this spot from SwerveModule
      // and update the strings so that they show as "Tuning/..." for each of the
      // values.
      // we need to store the results of the getnumber calls in function level
      // variables so they can be passed into the updatePID call.


      // this line is getting the selected subsystem from Elastic and sending the PID
      // values to that subsystem.
      ((ITunable) SmartDashboard.getData("Tuning/Selection")).updatePID(kp, ki, kd);
    }
  }
}

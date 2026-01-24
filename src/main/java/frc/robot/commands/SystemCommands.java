package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
// Work on calling buttons next week.
public class SystemCommands {
    public Command intakeBall;
    public Command outtakeBall;
    public Command shootBallFromGround;
    public Command shootBallFromHopper;

    public SystemCommands(IntakeSubsystem intake, TransferSubsystem transfer, ShooterSubsystem shooter) {
        // Takes in balls to use later.
        intakeBall = new FunctionalCommand(() -> {
            intake.startIntake();
            transfer.startIntake();
        }, () -> {
        }, interrupted -> {
            intake.stop();
            transfer.stopMotors();
        }, () -> {
            return intake.isHopperFull();
        }, intake, transfer);

        // spits out balls from inside the robot.
        outtakeBall = new FunctionalCommand(() -> {
            intake.startOuttake();
            transfer.outTake();
        }, () -> {
        }, interrupted -> {
            intake.stop();
            transfer.stopMotors();
        }, () -> {
            return transfer.isHopperEmpty();
        }, intake, transfer);

        // picks balls from intake and skips hopper to fire.
        shootBallFromGround = new FunctionalCommand(() -> {
            intake.startIntake();
            transfer.toLauncher();
        }, () -> {
            shooter.setMotorSpeed(.5); // TODO pick a speed with care.
        }, interrupted -> {
            intake.stop();
            transfer.stopMotors();
        }, () -> {
            return false;
        }, intake, transfer, shooter);

        // Takes balls from hopper and shoots them.
        shootBallFromHopper = new FunctionalCommand(() -> {
            transfer.toLauncher();
        }, () -> {
            shooter.setMotorSpeed(.5); // TODO pick speed with care or rig to speed distance calculations.
        }, interrupted -> {
            transfer.stopMotors();
        }, () -> {
            return transfer.isHopperEmpty();
        }, transfer, shooter);

    }

}

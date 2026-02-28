package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// Work on calling buttons next week.
public class SystemCommands {
    public Command intakeBall;
    public Command outtakeBall;
    public Command shootBallFromGround;
    public Command shootBallFromHopper;
    private Command shooting;
    private Command transferToShooter;
    private Command rollOverIntake;
    private Command rollOverOuttake;

    public SystemCommands(IntakeSubsystem intake, TransferSubsystem transfer, ShooterSubsystem shooter) {
        // Takes in balls to use later.
        rollOverIntake = new FunctionalCommand(
                shooter::intake,
                () -> {
                },
                (interrupted) -> {
                    shooter.stopMotor();
                },
                () -> false,
                shooter);

        rollOverOuttake = new FunctionalCommand(
                shooter::intake,
                () -> {
                },
                (interrupted) -> {
                    shooter.stopMotor();
                },
                () -> false,
                shooter);        
        // -------------------------------------------------------------------------------------------
        intakeBall = new ParallelCommandGroup(new FunctionalCommand(() -> {
            intake.startIntake();
            transfer.startIntake();
        }, () -> {
        }, interrupted -> {
            intake.stop();
            transfer.stopMotors();
        }, () -> {
            return intake.isHopperFull();
        }, intake, transfer),
                // ---------------------------------------------
                rollOverIntake);

        // spits out balls from inside the robot.
        outtakeBall = new ParallelCommandGroup(new FunctionalCommand(() -> {
            intake.startOuttake();
            transfer.outTake();
        }, () -> {
        }, interrupted -> {
            intake.stop();
            transfer.stopMotors();
        }, () -> {
            return transfer.isHopperEmpty();
        }, intake, transfer), 
                //-----------------------------------------------
                rollOverOuttake);
                

        shooting = new FunctionalCommand(
                () -> {
                    shooter.startFiring();
                },
                () -> {
                },
                (interrupted) -> {
                    shooter.startIdle();
                },
                () -> false,
                shooter);

        // picks balls from intake and skips hopper to fire.
        shootBallFromGround = new FunctionalCommand(() -> {
            intake.startIntake();
            transfer.toLauncher();
        }, () -> {
            shooter.startFiring();
        }, interrupted -> {
            intake.stop();
            transfer.stopMotors();
            shooter.startIdle();
        }, () -> {
            return false;
        }, intake, transfer, shooter);

        // Takes balls from hopper and shoots them.
        transferToShooter = new FunctionalCommand(() -> {
            transfer.toLauncher();
        }, () -> {
        }, interrupted -> {
            transfer.stopMotors();
        }, () -> {
            return transfer.isHopperEmpty();
        }, transfer);

        shootBallFromHopper = new ParallelCommandGroup(shooting,
                new SequentialCommandGroup(
                        new ParallelRaceGroup( new WaitCommand(2), 
                            new FunctionalCommand(
                                transfer::agitate, 
                                () -> {}, 
                                (interrupted) -> {transfer.stopMotors();}, 
                                () -> false)),

                        transferToShooter));
        
    }

}

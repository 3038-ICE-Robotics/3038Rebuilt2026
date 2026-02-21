package frc.robot.subsystems;

import java.nio.file.DirectoryIteratorException;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private SparkMax climbLeft;
    private SparkMax climbRight;
    private RelativeEncoder climbHeightR;
    private RelativeEncoder climbHeightL;
    private double retractPosition;
    private double extendPosition;
    private Command extL;
    private Command extR;
    private Command rtcL;
    private Command rtcR;
    public Command extend;
    public Command retract;
    public DigitalInput rightHome;
    public DigitalInput leftHome;

    public ClimberSubsystem() {
        climbLeft = new SparkMax(Constants.MotorIDs.ClimbLeft, MotorType.kBrushless);
        climbRight = new SparkMax(Constants.MotorIDs.ClimbRight, MotorType.kBrushless);
        climbHeightR = climbRight.getEncoder();
        climbHeightL = climbLeft.getEncoder();
        rightHome = new DigitalInput(Constants.DigitalChannels.RightClimbHome);
        leftHome = new DigitalInput(Constants.DigitalChannels.LeftClimbHome);
        // moves arm down.
        rtcL = new FunctionalCommand(() -> {
            climbLeft.set(-Constants.Climb.ClimbSpeed);
        }, () -> {
        }, interrupted -> {
            climbLeft.set(0);
            climbHeightL.setPosition(0);
        }, () -> !leftHome.get());
//---------------------------------------------------------------
        rtcR = new FunctionalCommand(() -> {
            climbRight.set(-Constants.Climb.ClimbSpeed);
        }, () -> {
        }, interrupted -> {
            climbRight.set(0);
            climbHeightR.setPosition(0);
        }, () -> !rightHome.get());
//---------------------------------------------------------------
        retract = new ParallelCommandGroup(rtcL, rtcR);
//---------------------------------------------------------------
        extL = new FunctionalCommand(() -> {
            climbLeft.set(Constants.Climb.ClimbSpeed);
        }, () -> {
        }, interrupted -> {
            climbLeft.set(0);
        }, () -> climbHeightL.getPosition() > Constants.Climb.ExtendHeight);
//---------------------------------------------------------------
        extR = new FunctionalCommand(() -> {
            climbRight.set(Constants.Climb.ClimbSpeed);
        }, () -> {
        }, interrupted -> {
            climbRight.set(0);
        }, () -> climbHeightR.getPosition() > Constants.Climb.ExtendHeight);
//---------------------------------------------------------------
        extend = new ParallelCommandGroup(extL, extR);
    }

    // public double getCurrentHeight() {
    //     return climbHeight.getPosition();
    // }

    // public void setHome() {
    //     retractPosition = getCurrentHeight();
    // }

    public void setSpeed(double speed) {
        climbRight.set(speed);
        climbLeft.set(speed);
    }

    // public boolean isHome() {
    //     return !rightHome.get();
    // }

    // public boolean isExtended() {
    //     return climbHeight.getPosition() > 96;
    // }

    public void periodic() {
SmartDashboard.putBoolean("Climber/Climber Home L", !leftHome.get());
    }

}

package frc.robot.subsystems;

import java.nio.file.DirectoryIteratorException;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private SparkMax climberPrime;
    private SparkMax climberFollow;
    private RelativeEncoder climbHeight;
    private double retractPosition;
    private double extendPosition;
    public Command extend;
    public Command retract;

    public ClimberSubsystem() {
        climberPrime = new SparkMax(Constants.MotorIDs.ClimbPrime, MotorType.kBrushless);
        climberFollow = new SparkMax(Constants.MotorIDs.ClimbFollow, MotorType.kBrushless);
        climbHeight = climberPrime.getAlternateEncoder();
        //moves arm down.
        retract = new FunctionalCommand(() -> {
            setSpeed(-.5);
        }, () -> {
        }, interrupted -> {
            setSpeed(0);
        }, this::isHome);
        //moves arm up.
        extend = new FunctionalCommand(() -> {
            setSpeed(.5);
        }, () -> {
        }, interrupted -> {
            setSpeed(0);
        }, this::isExtended);
        // TODO: Make use of constant for the channel
    }

    public double getCurrentHeight() {
        return climbHeight.getPosition();
    }

    public void setHome() {
        retractPosition = getCurrentHeight();
    }

    public void setSpeed(double speed) {
        climberPrime.set(speed);
    }

    public boolean isHome() {
        return climberPrime.getReverseLimitSwitch().isPressed();
    }

    public boolean isExtended() {
        return climberPrime.getForwardLimitSwitch().isPressed();
    }
    public void periodic() {
        
    }

}

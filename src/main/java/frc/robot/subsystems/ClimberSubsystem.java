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
    private SparkMax climbLeft;
    private SparkMax climbRight;
    private RelativeEncoder climbHeight;
    private double retractPosition;
    private double extendPosition;
    public Command extend;
    public Command retract;

    public ClimberSubsystem() {
        climbLeft = new SparkMax(Constants.MotorIDs.ClimbLeft, MotorType.kBrushless);
        climbRight = new SparkMax(Constants.MotorIDs.ClimbRight, MotorType.kBrushless);
        climbHeight = climbLeft.getAlternateEncoder();
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
        climbLeft.set(speed);
    }

    public boolean isHome() {
        return climbLeft.getReverseLimitSwitch().isPressed();
    }

    public boolean isExtended() {
        return climbLeft.getForwardLimitSwitch().isPressed();
    }
    public void periodic() {
        
    }

}

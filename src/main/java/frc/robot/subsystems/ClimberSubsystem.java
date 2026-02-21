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
    public DigitalInput rightHome;

    public ClimberSubsystem() {
        climbLeft = new SparkMax(Constants.MotorIDs.ClimbLeft, MotorType.kBrushless);
        climbRight = new SparkMax(Constants.MotorIDs.ClimbRight, MotorType.kBrushless);
        climbHeight = climbRight.getEncoder();
        rightHome = new DigitalInput(Constants.DigitalChannels.RightClimbHome);
        //moves arm down.
        retract = new FunctionalCommand(() -> {
            setSpeed(-.01);
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
        climbRight.set(speed);
    }

    public boolean isHome() {
        return rightHome.get();
    }

    public boolean isExtended() {
        return true;
    }
    public void periodic() {
        
    }

}

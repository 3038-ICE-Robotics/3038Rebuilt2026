package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private SparkMax climberPrime;
    private SparkMax climberFollow;
    private RelativeEncoder climbHeight;
    private double retractPosition;
    private double extendPosition;
    private DigitalInput homeSwitch;

    public ClimberSubsystem() {
        climberPrime = new SparkMax(Constants.MotorIDs.ClimbPrime, MotorType.kBrushless);
        climberFollow = new SparkMax(Constants.MotorIDs.ClimbFollow, MotorType.kBrushless);
        climbHeight = climberPrime.getAlternateEncoder();
        homeSwitch = new DigitalInput(0);
        //TODO: Make use of constant for the channel
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
        return homeSwitch.get();
    }

}

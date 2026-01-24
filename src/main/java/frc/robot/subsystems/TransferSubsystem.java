package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TransferSubsystem extends SubsystemBase {
    private SparkFlex frontMotor;
    private SparkFlex backMotor;
    private DigitalInput emptySensor;
    private double intakeSpeed = .5;
    private double outTakeSpeed = -.5;
    private double[] ampHistory = new double[25];
    private double standardAmp = 0;
    private int nextIndex = 0;
    private double ampSum = 0;
    private double ampThreshold = 10;

    public TransferSubsystem() {
        frontMotor = new SparkFlex(Constants.MotorIDs.IntakeFollow, MotorType.kBrushless);
        backMotor = new SparkFlex(Constants.MotorIDs.Transfer, MotorType.kBrushless);
        emptySensor = new DigitalInput(Constants.DigitalChannels.HopperEmpty);
    }

    public void startIntake() {
        frontMotor.set(intakeSpeed);
        backMotor.set(intakeSpeed);
    }

    public void stopMotors() {
        frontMotor.set(0);
        backMotor.set(0);
    }

    public void toLauncher() {
        frontMotor.set(intakeSpeed);
        backMotor.set(outTakeSpeed);
    }

    public void outTake() {
        frontMotor.set(outTakeSpeed);
        backMotor.set(outTakeSpeed);
    }

    public boolean isHopperEmpty() {
        return emptySensor.get();
    }

    public boolean isBallStuck() {
        ampSum = 0;
        for (double d : ampHistory) {
            ampSum += d;
        }
        return ampSum >= ampThreshold;
    }

    @Override
    public void periodic() {
        ampHistory[nextIndex] = standardAmp - backMotor.getOutputCurrent();
        nextIndex = (nextIndex + 1) % ampHistory.length;
    }
    
}

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TransferSubsystem extends SubsystemBase {
    private SparkFlex TransferLow;
    private SparkFlex TransferHigh;
   // private DigitalInput emptySensor;
    private double intakeSpeed = .5;
    private double outTakeSpeed = -.5;
    private double[] ampHistory = new double[25];
    private double standardAmp = 0;
    private int nextIndex = 0;
    private double ampSum = 0;
    //TODO: We need to actually test this to determine what a good threshold is.
    private double ampThreshold = 10;

    public TransferSubsystem() {
        TransferLow = new SparkFlex(Constants.MotorIDs.TransferLow, MotorType.kBrushless);
        TransferHigh = new SparkFlex(Constants.MotorIDs.TransferHigh, MotorType.kBrushless);
        // = new DigitalInput(Constants.DigitalChannels.HopperEmpty);
    }

    public void startIntake() {
        TransferLow.set(intakeSpeed - 0.1);
        TransferHigh.set(intakeSpeed - 0.3);
    }

    public void stopMotors() {
        TransferLow.set(0);
        TransferHigh.set(0);
    }

    public void agitate(){
        TransferHigh.set(intakeSpeed);
    }

    public void toLauncher() {
        TransferLow.set(intakeSpeed);
        TransferHigh.set(outTakeSpeed - 0.3);
    }

    public void outTake() {
        TransferLow.set(outTakeSpeed);
        TransferHigh.set(outTakeSpeed);
    }

    //TODO: Make sure mechanical actually gives us a breakbeam sensor to read for this.
    public boolean isHopperEmpty() {
        return false;//emptySensor.get();
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
        ampHistory[nextIndex] = standardAmp - TransferHigh.getOutputCurrent();
        nextIndex = (nextIndex + 1) % ampHistory.length;
    }
    
}

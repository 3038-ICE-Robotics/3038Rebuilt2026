package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private SparkFlex intakePrime;
    private DigitalInput hopperFullSensor;

    private SparkBaseConfig config;

    public IntakeSubsystem() {
        intakePrime = new SparkFlex(Constants.MotorIDs.IntakePrime, null);
        hopperFullSensor = new DigitalInput(Constants.DigitalChannels.HopperFull);
        config = new SparkFlexConfig();
        config
                .smartCurrentLimit(Constants.NeoVortex.StallCurrent)
                .idleMode(IdleMode.kCoast);
        intakePrime.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void startIntake() {
        intakePrime.set(Constants.MotorSpeeds.IntakeSpeed);
    }

    public void startOuttake() {
        intakePrime.set(-Constants.MotorSpeeds.IntakeSpeed);
    }

    public void stop() {
        intakePrime.set(0);
    }

    public boolean isHopperFull() {
        return !hopperFullSensor.get();
    }
    
    @Override
    public void periodic() {
        int numberBoolean = isHopperFull()?1:0;
        SmartDashboard.putNumber("Hopper Full Status", numberBoolean);
    }
}

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private SparkBaseConfig config;
    private double targetVelocity = 0;
    private final SparkFlex shooterPrime;
    private final SparkFlex shooterFollow;
    private SparkClosedLoopController VelocityControl;

    public ShooterSubsystem() {
        shooterPrime = new SparkFlex(Constants.MotorIDs.ShooterPrime, null);
        shooterFollow = new SparkFlex(Constants.MotorIDs.ShooterFollow, null);
        config = new SparkFlexConfig();
        config
                .smartCurrentLimit(50)
                .idleMode(IdleMode.kCoast)
                .follow(shooterPrime).closedLoop.pid(0.01, 0, 0.001);
        shooterPrime.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        shooterFollow.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        VelocityControl = shooterPrime.getClosedLoopController();
    }

    public double getCurrentSpeed() {
        return VelocityControl.getSetpoint();
    }

    public void setMotorSpeed(double speed) {
        VelocityControl.setSetpoint(speed, ControlType.kVelocity);
    }

    public void stopMotor() {
        setMotorSpeed(0);
    }

    @Override
    public void periodic() {
        // This runs every 20ms
        // Add any periodic updates here
    }
}

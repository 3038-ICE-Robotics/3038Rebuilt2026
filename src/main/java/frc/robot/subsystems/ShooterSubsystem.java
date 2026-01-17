package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.jni.ArmFeedforwardJNI;
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
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0);

    public ShooterSubsystem() {
        shooterPrime = new SparkFlex(Constants.MotorIDs.ShooterPrime, null);
        shooterFollow = new SparkFlex(Constants.MotorIDs.ShooterFollow, null);
        config = new SparkFlexConfig();
        config
                .smartCurrentLimit(Constants.NeoVortex.StallCurrent)
                .idleMode(IdleMode.kCoast)
                .follow(shooterPrime).closedLoop.pid(0.01, 0, 0.001, ClosedLoopSlot.kSlot0);
        shooterPrime.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        shooterFollow.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        VelocityControl = shooterPrime.getClosedLoopController();

    }

    public double getCurrentSpeed() {
        return VelocityControl.getSetpoint();
    }

    public void setMotorSpeed(double speed) {
        VelocityControl.setSetpoint(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward.calculate(speed));
    }

    public void stopMotor() {
        setMotorSpeed(0);
    }

    public boolean isStalled() {
        return shooterPrime.getOutputCurrent() >= Constants.NeoVortex.StallCurrent;

    }

    @Override
    // This runs every 20ms
    // Add any periodic updates here
    public void periodic() {
        // This runs constantly!

        // 1. Update Dashboard
        SmartDashboard.putNumber("Prime shooter Temp", shooterPrime.getMotorTemperature());
        SmartDashboard.putNumber("Prime Shooter Current", shooterPrime.getOutputCurrent());
        SmartDashboard.putNumber("Follow shooter Temp", shooterFollow.getMotorTemperature());
        SmartDashboard.putNumber("Follow Shooter Current", shooterFollow.getOutputCurrent());
        // 2. Continuous Safety Checks
        if (shooterPrime.getMotorTemperature() > 80) {
            System.out.println("🔥 INTAKE OVERHEATING! STOPPING!");
            shooterPrime.set(0);
        }
    }
}

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.StateOfRobot;

public class ShooterSubsystem extends SubsystemBase {
    private SparkBaseConfig configF;
    private SparkBaseConfig configP;
    private double targetVelocity = 0;
    private final SparkFlex shooterPrime;
    private final SparkFlex shooterFollow;
    private SparkClosedLoopController VelocityControl;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.0015, 0.0015);
    private Supplier<Pose2d> robotPoint;

    private enum ShooterModes {
        IDLE, FIRING, STOP, INTAKE
    }

    private ShooterModes shooterSelect;
    public Supplier <Double> speedControl;

    private PIDController shooterPID;
    private LinearFilter filter = LinearFilter.singlePoleIIR(0.05, 0.02);
    public Supplier <Double> distanceControl;

    public ShooterSubsystem(Supplier<Pose2d> robotPosition) {
        shooterPrime = new SparkFlex(Constants.MotorIDs.ShooterPrime, MotorType.kBrushless);
        shooterFollow = new SparkFlex(Constants.MotorIDs.ShooterFollow, MotorType.kBrushless);
        shooterPID = new PIDController(0.0035, 0, 0.001);
        configP = new SparkFlexConfig();
        configP.closedLoop.pid(0.1, 0, 0.001, ClosedLoopSlot.kSlot0);
        configF = new SparkFlexConfig();
        configF.follow(shooterPrime, true);
        shooterPrime.configure(configP, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        shooterFollow.configure(configF, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        VelocityControl = shooterPrime.getClosedLoopController();
        robotPoint = robotPosition;
        shooterSelect = ShooterModes.IDLE;

    }

    public double getCurrentSpeed() {
        return shooterPrime.getEncoder().getVelocity();
    }

    public void setMotorSpeed(double speed) {
        double shooterPIDcalculated = shooterPID.calculate(filter.calculate(getCurrentSpeed()) - speed);
        // VelocityControl.setSetpoint(speed, ControlType.kVelocity,
        //         ClosedLoopSlot.kSlot0, feedforward.calculate(speed));

        //12 * speed / 400
        shooterPrime.setVoltage(MathUtil.clamp(shooterPIDcalculated + feedforward.calculate(speed), 0, 12));
SmartDashboard.putNumber("Shooter/PID", shooterPIDcalculated);
    }

    public void stopMotor() {
        setMotorSpeed(0);
        shooterSelect = ShooterModes.STOP;
    }

    public void startFiring() {
        shooterSelect = ShooterModes.FIRING;
    }

    public void startIdle() {
        shooterSelect = ShooterModes.IDLE;
    }

    public boolean isStalled() {
        return shooterPrime.getOutputCurrent() >= Constants.NeoVortex.StallCurrent;

    }

    @Override
    // This runs every 20ms
    // Add any periodic updates here
    public void periodic() {
        // This runs constantly!

        // // 1. Update Dashboard
        // SmartDashboard.putNumber("Shooter/Prime shooter Temp", shooterPrime.getMotorTemperature());
        // SmartDashboard.putNumber("Shooter/Prime Shooter Current", shooterPrime.getOutputCurrent());
        // SmartDashboard.putNumber("Shooter/Follow shooter Temp", shooterFollow.getMotorTemperature());
        // SmartDashboard.putNumber("Shooter/Follow Shooter Current", shooterFollow.getOutputCurrent());
        // 2. Continuous Safety Checks
        if (shooterPrime.getMotorTemperature() > 80) {
            System.out.println("🔥 Shooter OVERHEATING! STOPPING!");
            shooterPrime.set(0);
        }
        double distanceFromTarget = StateOfRobot.distanceBetweenTargetAnd(robotPoint.get())*Constants.MetersToFeet*12;
        if (DriverStation.isTest()) {
            distanceFromTarget = distanceControl.get();
        }
        double targetSpeed = 0;
        SmartDashboard.putNumber("Shooter/Distance To Target", distanceFromTarget);

        switch (shooterSelect) {
            case FIRING:
                targetSpeed = StateOfRobot.getSpeedFromDistance(distanceFromTarget);
                break;
            case IDLE:
                targetSpeed = 100;
                break;
            case STOP:
                targetSpeed = 0;
                break;
            case INTAKE:
                targetSpeed = 100;
                break;
        }
        if (DriverStation.isTest()) {
            targetSpeed = speedControl.get();
        }
        setMotorSpeed(targetSpeed);
        // SmartDashboard.putNumber("Shooter/Accum Error", VelocityControl.getIAccum());
        SmartDashboard.putNumber("Shooter/Target Motor Speed", targetSpeed);
        SmartDashboard.putNumber("Shooter/Physical Motor Speed", getCurrentSpeed());
        SmartDashboard.putString("Shooter/Select", shooterSelect.toString());
    }


    public void intake() {
        shooterSelect = ShooterModes.INTAKE;
    }

}

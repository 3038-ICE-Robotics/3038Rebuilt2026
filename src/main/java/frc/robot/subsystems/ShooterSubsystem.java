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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.0075);
    private Supplier<Pose2d> robotPoint;

    private enum ShooterModes {
        IDLE, FIRING, STOP, INTAKE
    }

    private ShooterModes shooterSelect;

    public ShooterSubsystem(Supplier<Pose2d> robotPosition) {
        shooterPrime = new SparkFlex(Constants.MotorIDs.ShooterPrime, MotorType.kBrushless);
        shooterFollow = new SparkFlex(Constants.MotorIDs.ShooterFollow, MotorType.kBrushless);
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
        // VelocityControl.setSetpoint(speed, ControlType.kVelocity,
        //         ClosedLoopSlot.kSlot0, feedforward.calculate(speed));
        shooterPrime.setVoltage(12 * speed / 1000);

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

        // 1. Update Dashboard
        SmartDashboard.putNumber("Shooter/Prime shooter Temp", shooterPrime.getMotorTemperature());
        SmartDashboard.putNumber("Shooter/Prime Shooter Current", shooterPrime.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/Follow shooter Temp", shooterFollow.getMotorTemperature());
        SmartDashboard.putNumber("Shooter/Follow Shooter Current", shooterFollow.getOutputCurrent());
        // 2. Continuous Safety Checks
        if (shooterPrime.getMotorTemperature() > 80) {
            System.out.println("🔥 Shooter OVERHEATING! STOPPING!");
            shooterPrime.set(0);
        }
        double distanceFromTarget = StateOfRobot.distanceBetweenTargetAnd(robotPoint.get());
        double targetSpeed = 0;
        SmartDashboard.putNumber("Shooter/Distance To Target", distanceFromTarget);

        switch (shooterSelect) {
            case FIRING:
                targetSpeed = getSpeedFromDistance(distanceFromTarget);
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
        if(DriverStation.isTest()){
            targetSpeed/=2;
        }
        setMotorSpeed(targetSpeed);
        // SmartDashboard.putNumber("Shooter/Accum Error", VelocityControl.getIAccum());
        SmartDashboard.putNumber("Shooter/Target Motor Speed", targetSpeed);
        SmartDashboard.putNumber("Shooter/Physical Motor Speed", getCurrentSpeed());
        SmartDashboard.putString("Shooter/Select", shooterSelect.toString());
    }

    private double getSpeedFromDistance(double distance) {
        int rightIndex = -1;
        for (int i = 0; i < Constants.AimBotData.distancesToHub.length; i++) {
            if (distance < Constants.AimBotData.distancesToHub[i]) {
                rightIndex = i;
                break;
            }
        }
        if (rightIndex == 0) {
            return Constants.AimBotData.shooterSpeeds[0];
        }
        if (rightIndex == -1) {
            return Constants.AimBotData.shooterSpeeds[Constants.AimBotData.distancesToHub.length - 1];
        }
        double percent = MathUtil.inverseInterpolate(Constants.AimBotData.distancesToHub[rightIndex - 1],
                Constants.AimBotData.distancesToHub[rightIndex], distance);
        return MathUtil.interpolate(Constants.AimBotData.shooterSpeeds[rightIndex - 1],
                Constants.AimBotData.shooterSpeeds[rightIndex], percent);
    }

    public void intake() {
        shooterSelect = ShooterModes.INTAKE;
    }

}

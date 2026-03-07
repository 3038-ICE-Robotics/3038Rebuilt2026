package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RearSubsystem extends SubsystemBase {
    private SparkFlex rearLeft;
    private SparkFlex rearRight;
    private SparkFlex rearIntake;

    private SparkBaseConfig configL;
    private SparkBaseConfig configR;
    private SparkBaseConfig configIntake;

    private boolean agitateUp = true;

    private final AbsoluteEncoder rearEncoder; // located on right encoder

    public RearSubsystem() {
        rearLeft = new SparkFlex(Constants.MotorIDs.RearLeft, MotorType.kBrushless);
        rearRight = new SparkFlex(Constants.MotorIDs.RearRight, MotorType.kBrushless);
        rearIntake = new SparkFlex(Constants.MotorIDs.RearIntake, MotorType.kBrushless);
        rearEncoder = rearRight.getAbsoluteEncoder();
        configL = new SparkFlexConfig();
        configR = new SparkFlexConfig();
        configIntake = new SparkFlexConfig();
        configIntake
                .smartCurrentLimit(Constants.NeoVortex.StallCurrent)
                .idleMode(IdleMode.kCoast);
        configL.follow(rearRight, true);
        rearLeft.configure(configL, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        rearRight.configure(configR, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        rearIntake.configure(configIntake, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void startIntake() {
        rearIntake.set(Constants.MotorSpeeds.RearIntakeSpeed);
    }

    public void stopIntake() {
        rearIntake.set(0);
    }

    public void extend() {
        rearRight.set(Constants.MotorSpeeds.RearSpeed);
    }

    public void retract() {
        rearRight.set(-Constants.MotorSpeeds.RearSpeed);
    }

    public void stop() {
        rearRight.set(0);
    }

    public void agitate() {
        double position = getAdjustedEncoder();
        if (agitateUp) {
            if (position <= Constants.HippoData.AgitateLimit) {
                agitateUp = false;
            }
        } else {
            if (position >= Constants.HippoData.ExtendLimit) {
                agitateUp = true;
            }
        }
        rearRight.set((agitateUp? -1:1) * Constants.MotorSpeeds.RearSpeed);
    }

    public boolean isRetracted() {
        return getAdjustedEncoder() <= Constants.HippoData.RetractLimit;
    }

    public boolean isExtended() {
        return getAdjustedEncoder() >= Constants.HippoData.ExtendLimit;
    }

    private double getAdjustedEncoder() {
        return MathUtil.inputModulus(rearEncoder.getPosition() + 0.5, 0, 1);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Rear/Position", getAdjustedEncoder());
        // SmartDashboard.putBoolean("Rear/isExtended", isExtended());
        // SmartDashboard.putBoolean("Rear/isRetracted", isRetracted());
        SmartDashboard.putNumber("Rear/Intake Velocity", rearIntake.getEncoder().getVelocity());
    }
}

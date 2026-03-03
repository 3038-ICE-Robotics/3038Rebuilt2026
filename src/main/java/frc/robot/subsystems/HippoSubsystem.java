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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HippoSubsystem extends SubsystemBase {
    private SparkFlex hippoLeft;
    private SparkFlex hippoRight;
    private SparkFlex hippoIntake;

    private SparkBaseConfig configL;
    private SparkBaseConfig configR;
    private SparkBaseConfig configIntake;

     private final AbsoluteEncoder hippoEncoder; //located on right encoder

    public HippoSubsystem() {
        hippoLeft = new SparkFlex(Constants.MotorIDs.HippoLeft, MotorType.kBrushless);
        hippoRight = new SparkFlex(Constants.MotorIDs.HippoRight, MotorType.kBrushless);
        hippoIntake = new SparkFlex(Constants.MotorIDs.HippoIntake, MotorType.kBrushless);
        hippoEncoder = hippoRight.getAbsoluteEncoder();
        configL = new SparkFlexConfig();
        configR = new SparkFlexConfig();
        configIntake = new SparkFlexConfig();
        configIntake
                .smartCurrentLimit(Constants.NeoVortex.StallCurrent)
                .idleMode(IdleMode.kCoast);
        configR.follow(hippoLeft, true);
        hippoLeft.configure(configL, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        hippoRight.configure(configR, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        hippoIntake.configure(configIntake, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void startIntake() {
        hippoIntake.set(Constants.MotorSpeeds.HippoIntakeSpeed);
    }

    public void stopIntake() {
        hippoIntake.set(0);
    }

    public void extend() {
        hippoLeft.set(Constants.MotorSpeeds.HippoSpeed);
    }

    public void retract() {
        hippoLeft.set(-Constants.MotorSpeeds.HippoSpeed);
    }

    public void stop() {
        hippoLeft.set(0);
    }

    @Override
    public void periodic() {

    }
}

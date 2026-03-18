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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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

    public Command extendLeft;
    public Command MaintainExtend;

    public Command extendRight;
    public Command extendRightTest;

    public Command retractLeft;
    public Command retractRight;

    private AbsoluteEncoder rearRightEncoder; // located on right encoder
    private AbsoluteEncoder rearLeftEncoder;

    public RearSubsystem() {
        rearLeft = new SparkFlex(Constants.MotorIDs.RearLeft, MotorType.kBrushless);
        rearRight = new SparkFlex(Constants.MotorIDs.RearRight, MotorType.kBrushless);
        rearIntake = new SparkFlex(Constants.MotorIDs.RearIntake, MotorType.kBrushless);
        rearRightEncoder = rearRight.getAbsoluteEncoder();
        rearLeftEncoder = rearLeft.getAbsoluteEncoder();
        configL = new SparkFlexConfig();
        configL.inverted(true);
        configR = new SparkFlexConfig();
        configIntake = new SparkFlexConfig();
        configIntake
                .smartCurrentLimit(Constants.NeoVortex.StallCurrent)
                .idleMode(IdleMode.kCoast);
        rearLeft.configure(configL, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        rearRight.configure(configR, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        rearIntake.configure(configIntake, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        extendLeft = new FunctionalCommand(
                () -> rearLeft.set(Constants.MotorSpeeds.RearSpeed),
                () -> {
                },
                (interrupted) -> rearLeft.set(0),
                () -> isExtendedL());
        extendRight = new FunctionalCommand(
                () -> rearRight.set(Constants.MotorSpeeds.RearSpeed),
                () -> {
                },
                (interrupted) -> rearRight.set(0),
                () -> isExtendedR());
        MaintainExtend = new FunctionalCommand(
                () -> {},
                this::maintainExtend,
                (interrupted) -> {
                    rearLeft.set(0);
                    rearRight.set(0);
                },
                () -> false
        );
        extendRightTest = new FunctionalCommand(
                () -> rearRight.set(Constants.MotorSpeeds.RearSpeed),
                () -> {
                },
                (interrupted) -> rearRight.set(0),
                () -> isExtendedR());

        retractLeft = new FunctionalCommand(
                () -> rearLeft.set(-Constants.MotorSpeeds.RearSpeed),
                () -> {
                },
                (interrupted) -> rearLeft.set(0),
                () -> isRetractedL());
        retractRight = new FunctionalCommand(
                () -> rearRight.set(-Constants.MotorSpeeds.RearSpeed),
                () -> {
                },
                (interrupted) -> rearRight.set(0),
                () -> isRetractedR());
    }

    public void startIntake() {
        rearIntake.set(Constants.MotorSpeeds.RearIntakeSpeed);
    }

    public void stopIntake() {
        rearIntake.set(0);
    }

    public void extend() {
        rearRight.set(Constants.MotorSpeeds.RearSpeed);
        rearLeft.set(Constants.MotorSpeeds.RearSpeed + 0.1);
    }

    public void retract() {
        rearRight.set(-Constants.MotorSpeeds.RearSpeed);
        rearLeft.set(-Constants.MotorSpeeds.RearSpeed);
    }

    public void stop() {
        rearRight.set(0);
        rearLeft.set(0);
    }

    private boolean isLeftPastBoundary() {
        return getAdjustedLeft() >= Constants.HippoData.AgitateLimitL;
    }

    private boolean isRightPastBoundary() {
        return getAdjustedRight() >= Constants.HippoData.AgitateLimitR;
    }

    public void agitate() {
        double leftSpeed = 0;
        double rightSpeed = 0;
        if (agitateUp) {
            if (!isLeftPastBoundary()) {
                leftSpeed = -Constants.MotorSpeeds.RearSpeed;
            }
            if (!isRightPastBoundary()) {
                rightSpeed = -Constants.MotorSpeeds.RearSpeed;
            }
            agitateUp = !(isLeftPastBoundary() && isRightPastBoundary());
        } else {
            if (!isExtendedL()) {
                leftSpeed = Constants.MotorSpeeds.RearSpeed;
            }
            if (!isExtendedR()) {
                rightSpeed = Constants.MotorSpeeds.RearSpeed;
            }
            agitateUp = (isExtendedL() && isExtendedR());
        }
        rearRight.set(rightSpeed / 2);
        rearLeft.set(leftSpeed / 2);
    }

    public void maintainExtend() {
        double leftSpeed = 0.0;
        double rightSpeed = 0.0;
        if (!isExtendedL()) {
            leftSpeed = Constants.MotorSpeeds.RearSpeed;
        }
        if (!isExtendedR()) {
            rightSpeed = Constants.MotorSpeeds.RearSpeed;
        }
        rearRight.set(rightSpeed / 2);
        rearLeft.set(leftSpeed / 2);
    }

    public boolean isRetractedL() {
        return getAdjustedLeft() >= Constants.HippoData.RetractLimitL;
    }

    public boolean isRetractedR() {
        return getAdjustedRight() >= Constants.HippoData.RetractLimitR;
    }

    public boolean isExtendedL() {
        return getAdjustedLeft() <= Constants.HippoData.ExtendLimitL;
    }

    public boolean isExtendedR() {
        return getAdjustedRight() <= Constants.HippoData.ExtendLimitR;
    }

    private double getAdjustedRight() {
        return MathUtil.inputModulus(-rearRightEncoder.getPosition(), 0, 1);
    }

    private double getAdjustedLeft() {
        return MathUtil.inputModulus(rearLeftEncoder.getPosition(), 0, 1);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Rear/PositionL", getAdjustedLeft());
        SmartDashboard.putNumber("Rear/PositionR", getAdjustedRight());
        SmartDashboard.putBoolean("Rear/isExtended", isExtendedL());
        SmartDashboard.putBoolean("Rear/isRetracted", isRetractedL());
        SmartDashboard.putNumber("Rear/Intake Velocity", rearIntake.getEncoder().getVelocity());
    }
}

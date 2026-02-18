package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
    private String moduleName;
    private double FoundOffset;
    // The mechanical bits
    private final SparkFlex driveMotor;
    private final SparkMax steerMotor;
    private final AbsoluteEncoder swerveEncoder;
    private final Rotation2d m_steerEncoderOffset;
    private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0, 0);
    private SimpleMotorFeedforward steerFF = new SimpleMotorFeedforward(0, 0);
    private SparkBaseConfig driveConfig;
    private SparkBaseConfig steerConfig;
    private AbsoluteEncoderConfig encoderConfig;
    // Shuffleboard stuff
    ShuffleboardTab debugInfo;
    // Variables
    /** The target wheel angle in rotations. [-.5, .5] */
    private double desiredSteerAngle;
    /** The target wheel speed in rotations per second */
    private double desiredDriveSpeed;
    // Controls
    private SparkClosedLoopController driveControl;
    private SparkClosedLoopController steerControl;
    // private PositionDutyCycle steerControl = new PositionDutyCycle(0);

    /**
     * Constructs a SwerveModule with a drive motor, steering motor, and steering
     * encoder.
     * Also takes a rotation 2d offset for directions and a canivore name.
     *
     *
     * @param driveMotorChannel   (drive motor id, integer)
     * @param steerMotorChannel   (steering motor id, integer)
     * @param steerEncoderChannel (steering encoder id, integer)
     * @param steerEncoderOffset  (how far the wheel is offset, rotation2d)
     */
    public SwerveModule(
            String moduleName,
            int driveMotorChannel,
            int steerMotorChannel,
            Rotation2d steerEncoderOffset) {
        this.moduleName = moduleName;
        driveMotor = new SparkFlex(driveMotorChannel, MotorType.kBrushless);
        steerMotor = new SparkMax(steerMotorChannel, MotorType.kBrushless);
        swerveEncoder = steerMotor.getAbsoluteEncoder();
        driveControl = driveMotor.getClosedLoopController();
        steerControl = steerMotor.getClosedLoopController();
        m_steerEncoderOffset = steerEncoderOffset;
        driveConfig = new SparkFlexConfig();
        steerConfig = new SparkMaxConfig();
        encoderConfig = new AbsoluteEncoderConfig();
        encoderConfig.zeroOffset(-m_steerEncoderOffset.getRotations());
        driveConfig.closedLoop.pid(1, 0, 0, ClosedLoopSlot.kSlot0);
        steerConfig.closedLoop.pid(0.3, 0, 0.01, ClosedLoopSlot.kSlot0);
        // Apply the configurations.
        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        steerMotor.configure(steerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        FoundOffset = (findOffset() * Constants.DriveTrain.SteerGearRatio);
        SmartDashboard.putNumber("PID/pvalue", Constants.DriveTrain.RotationkP);
        SmartDashboard.putNumber("PID/ivalue", Constants.DriveTrain.RotationkI);
        SmartDashboard.putNumber("PID/dvalue", Constants.DriveTrain.RotationkD);
    }

    // This section is the 'direct get' section. Everything that gets something
    // directly from a motor is in here.
    public SparkFlex getDriveMotor() {
        return driveMotor;
    }

    public SparkMax getSteerMotor() {
        return steerMotor;
    }

    /**
     * Returns the current encoder distance of the drive motor.
     *
     * @return The current distance of the drive motor in meters.
     */
    public double getDriveDistanceMeters() {
        return (driveMotor.getEncoder().getPosition() * Constants.DriveTrain.RotationsToMeters);
    }

    /**
     * Returns the current encoder angle of the steer motor.
     *
     * @return The current encoder angle of the steer motor.
     */
    public Rotation2d getRotation() {
        return Rotation2d.fromRotations(
                MathUtil.inputModulus(steerMotor.getEncoder().getPosition(), -0.5, 0.5));
    }

    /**
     * Returns the current encoder velocity of the drive motor.
     *
     * @return The current velocity of the drive motor in meters/second.
     */
    public double getSpeedMetersPerSecond() {
        return (driveMotor.getEncoder().getVelocity()
                * Constants.DriveTrain.RotationsToMeters);
    }

    /**
     * finds the current encoder position, it removes the current offset so we just
     * get the raw
     * position
     *
     * @return
     */
    public double findOffset() {
        return MathUtil.inputModulus(
                (swerveEncoder.getPosition() + m_steerEncoderOffset.getRotations()),
                -0.5,
                0.5);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeedMetersPerSecond(), getRotation());
    }

    /**
     * Returns the current position of the module. Includes the modules rotation and
     * the modules
     * distance driven.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistanceMeters(), getRotation());
    }

    // This is the direction section. It recives instructions and sets the desired
    // state of the swerve module.
    /**
     * Returns the target angle of the wheel.
     *
     * @return The target angle of the wheel in degrees.
     */
    public double getTargetAngle() {
        return desiredSteerAngle;
    }

    /**
     * Returns the target speed of the wheel.
     *
     * @return The target speed of the wheel in meters/second.
     */
    public double getTargetSpeedMetersPerSecond() {
        return desiredDriveSpeed;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {

        if (desiredState.angle == null) {
            DriverStation.reportWarning("Cannot set module angle to null.", true);
        }

        // Optimize the reference state to avoid spinning further than 90 degrees.
        desiredState.optimize(getRotation());

        desiredSteerAngle = MathUtil.inputModulus(desiredState.angle.getRotations(), -0.5, 0.5);
        SmartDashboard.putNumber("Optimized/Angle" + moduleName, desiredSteerAngle);
        desiredDriveSpeed = desiredState.speedMetersPerSecond / Constants.DriveTrain.RotationsToMeters;

        if (Math.abs(desiredDriveSpeed) <= 0.001) {
            // driveMotor.setControl(neutralControl);
        } else {
            // driveControl
            // .setSetpoint(desiredDriveSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot0,
            // driveFF.calculate(desiredDriveSpeed));

        }
        steerControl.setSetpoint(desiredSteerAngle * Constants.DriveTrain.SteerGearRatio, ControlType.kPosition,
                ClosedLoopSlot.kSlot0);
        // steerFF.calculate(desiredDriveSpeed));

    }

    public void periodic() {
        // TODO: Move this into the RobotContainer.updatePID function.
        double kp = SmartDashboard.getNumber("PID/pvalue", Constants.DriveTrain.RotationkP);
        double ki = SmartDashboard.getNumber("PID/ivalue", Constants.DriveTrain.RotationkI);
        double kd = SmartDashboard.getNumber("PID/dvalue", Constants.DriveTrain.RotationkD);
        // TODO: Move this into a new function called updatePID which takes in 3
        // parameters, kp, ki, and kd.
        steerConfig.closedLoop.pid(kp, ki, kd);
        steerMotor.configure(steerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // TODO: Use DriveSubsystem.updatePID as an example for the function name and parameters, but the implementation is just the 2 steerConfig and steerMotor lines.
}

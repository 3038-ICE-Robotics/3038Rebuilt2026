package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.path.GoalEndState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AimBotData;

public class StateOfRobot {
    public static Translation2d target;

    public enum TargetType {
        GOAL, ALLIANCE
    }

    public static TargetType targetType = TargetType.GOAL;
    public static Optional<Alliance> ally = DriverStation.getAlliance();
    public static boolean isAimAssistOn = false;
    public static PIDController aimAssistPID = new PIDController(AimBotData.RotationPID[0], AimBotData.RotationPID[1],
            AimBotData.RotationPID[2]);

    public static void toggleAimAssist() {
        if (isAimAssistOn) {
            isAimAssistOn = false;
            aimAssistPID.reset();
            aimAssistPID.enableContinuousInput(-.5, .5);
        } else {
            isAimAssistOn = true;
        }

    }

    public static double getAimBotRotation(Supplier<Rotation2d> desiredAngle, Supplier<Rotation2d> botPose2d) {
        SmartDashboard.putNumber("Angles/desired Angle", desiredAngle.get().getRotations());
        SmartDashboard.putNumber("Angles/desired Angle Degrees", desiredAngle.get().getDegrees());
        SmartDashboard.putNumber("Angles/pose 2d", botPose2d.get().getRotations());
        SmartDashboard.putNumber("Angles/pose 2d Degrees", botPose2d.get().getDegrees());

        // double value = MathUtil.clamp(
        //         aimAssistPID.calculate(desiredAngle.get().getRotations() - botPose2d.get().getRotations()), -1, 1);
        // double value = (
        //         desiredAngle.get().getRotations() - botPose2d.get().getRotations());
        // if (Math.abs(value) > 1) {
        //      value = Math.copySign(1-Math.abs(value), -value);
        // } 
        double value = aimAssistPID.calculate(botPose2d.get().getRotations(), desiredAngle.get().getRotations());
        SmartDashboard.putNumber("Angles/Value", value);
        return value;

    }

    public static double getSpeedFromDistance(double distance) {
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

    public static double distanceBetweenTargetAnd(Pose2d start) {
        double dx = target.getX() - start.getX();
        double dy = target.getY() - start.getY();
        if (targetType == TargetType.ALLIANCE) {
            return dx;
        } else {
            return Math.sqrt(dx * dx + dy * dy);
        }
    }

    public static double angleBetweenTargetAnd(Pose2d start) {
        double dx = target.getX() - start.getX();
        double dy = target.getY() - start.getY();
        if (targetType == TargetType.ALLIANCE) {
            dy = 0;
        }
        return Math.atan2(dy, dx);
    }

    public static void setTargetHUB() {
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                target = Constants.Field.RedHub;
            } else {
                target = Constants.Field.BlueHub;
            }
        }
        targetType = TargetType.GOAL;
    }

    public static void setTargetZONE() {
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                target = Constants.Field.RedZone;
            } else {
                target = Constants.Field.BlueZone;
            }
        }
        targetType = TargetType.ALLIANCE;
    }
}

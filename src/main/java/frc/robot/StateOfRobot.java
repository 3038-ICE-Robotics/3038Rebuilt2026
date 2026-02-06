package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.path.GoalEndState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
        } else {
            isAimAssistOn = true;
        }

    }
    public static double getAimBotRotation(Pose2d botPose2d) {
        return aimAssistPID.calculate(angelBetweenTargetAnd(botPose2d) - botPose2d.getRotation().getRadians());
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

    public static double angelBetweenTargetAnd(Pose2d start) {
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

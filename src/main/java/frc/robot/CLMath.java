package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class CLMath {
    public static Pose2d MultiplyPose(Pose2d startPose, double scale) {
        return new Pose2d(startPose.getX() * scale, startPose.getY() * scale, startPose.getRotation());
    }

    public static Pose2d DividePose(Pose2d startPose, double scale) {
        return new Pose2d(startPose.getX() / scale, startPose.getY() / scale, startPose.getRotation());
    }

    public static Pose2d AddPose(Pose2d Pose1, Pose2d Pose2) {
        return new Pose2d(Pose1.getX() + Pose2.getX(), Pose1.getY() + Pose2.getY(), Pose1.getRotation());
    }

    public static double DistanceFromOrigin3d(double X, double Y, double Z) {
        return Math.sqrt(X * X + Y * Y + Z * Z);
    }
}

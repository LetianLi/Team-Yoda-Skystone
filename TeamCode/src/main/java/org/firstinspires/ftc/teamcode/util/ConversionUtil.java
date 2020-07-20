package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

public class ConversionUtil {
    private static double metersInInch = 0.0254;
    private static double inchesInMeter = 1 / metersInInch;

    public static double metersToInches(double meters) {
        return meters * inchesInMeter;
    }

    public static double inchesToMeters(double inches) {
        return inches * metersInInch;
    }

    public static Pose2d toRRPose2d(Transform2d transform2d) {
        return new Pose2d(transform2d.getTranslation().getX(), transform2d.getTranslation().getY(), transform2d.getRotation().getRadians());
    }

    public static Transform2d toTransform2d(Pose2d pose2d) {
        return new Transform2d(new Translation2d(pose2d.getX(), pose2d.getY()), new Rotation2d(pose2d.getHeading()));
    }

    public static com.arcrobotics.ftclib.geometry.Pose2d toLibPose2d(Pose2d RRPose2d) {
        return new com.arcrobotics.ftclib.geometry.Pose2d(RRPose2d.getX(), RRPose2d.getY(), new Rotation2d(RRPose2d.getHeading()));
    }



    public static Pose2d inchesToMeters(Pose2d pose2d) {
        return new Pose2d(pose2d.vec().times(metersInInch), pose2d.getHeading());
    }

    public static Vector2d inchesToMeters(Vector2d vector2d) {
        return vector2d.times(metersInInch);
    }

    public static com.arcrobotics.ftclib.geometry.Pose2d inchesToMeters(com.arcrobotics.ftclib.geometry.Pose2d pose2d) {
        return new com.arcrobotics.ftclib.geometry.Pose2d(pose2d.getTranslation().times(metersInInch), pose2d.getRotation());
    }



    public static Pose2d metersToInches(Pose2d pose2d) {
        return new Pose2d(pose2d.vec().times(inchesInMeter), pose2d.getHeading());
    }

    public static Vector2d metersToInches(Vector2d vector2d) {
        return vector2d.times(inchesInMeter);
    }

    public static com.arcrobotics.ftclib.geometry.Pose2d metersToInches(com.arcrobotics.ftclib.geometry.Pose2d pose2d) {
        return new com.arcrobotics.ftclib.geometry.Pose2d(pose2d.getTranslation().times(inchesInMeter), pose2d.getRotation());
    }
}

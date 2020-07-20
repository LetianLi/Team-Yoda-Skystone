package org.firstinspires.ftc.teamcode.yoda_code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.util.ConversionUtil;
import org.jetbrains.annotations.NotNull;

import java.util.Locale;

/**
 * Formatted Bulk Read of CameraUpdate. Stores: Positiion, Velocity, Confidence
 */
public class CameraData {
    private final Pose2d robotPosition;
    private final Pose2d robotVelocity;
    private final T265Camera.PoseConfidence poseConfidence;
    private final int repeats;

    private final int repeatsThreshold = 1; // If repeats gets to this number, it is no longer unique

    public CameraData() {
        this.robotPosition = new Pose2d();
        this.robotVelocity = new Pose2d();
        this.poseConfidence = T265Camera.PoseConfidence.Failed;
        this.repeats = 0;
    }

    public CameraData(T265Camera.CameraUpdate cameraUpdate, CameraData previousData) {
        this.robotPosition = ConversionUtil.metersToInches(new Pose2d(cameraUpdate.pose.getTranslation().getX(), cameraUpdate.pose.getTranslation().getY(), cameraUpdate.pose.getRotation().getRadians()));
        this.robotVelocity = ConversionUtil.metersToInches(new Pose2d(cameraUpdate.velocity.vxMetersPerSecond, cameraUpdate.velocity.vyMetersPerSecond, cameraUpdate.velocity.omegaRadiansPerSecond));
        this.poseConfidence = cameraUpdate.confidence;

        if (this.equals(previousData)) this.repeats = previousData.repeats + 1;
        else this.repeats = 0;
    }

    public CameraData(Pose2d robotPosition, Pose2d robotVelocity, T265Camera.PoseConfidence poseConfidence) {
        this.robotPosition = robotPosition;
        this.robotVelocity = robotVelocity;
        this.poseConfidence = poseConfidence;
        this.repeats = 0;
    }


    public Pose2d getRobotPosition() {
        return robotPosition;
    }

    public Pose2d getRobotVelocity() {
        return robotVelocity;
    }

    public T265Camera.PoseConfidence getPoseConfidence() {
        return poseConfidence;
    }


    public double getX() {
        return robotPosition.getX();
    }

    public double getY() {
        return robotPosition.getY();
    }

    public double getHeading() {
        return robotPosition.getHeading();
    }

    public double getVelX() {
        return robotVelocity.getX();
    }

    public double getVelY() {
        return robotVelocity.getY();
    }

    public double getVelHeading() {
        return robotVelocity.getHeading();
    }

    public int getRepeats() {
        return repeats;
    }

    public boolean isUnique() {
        return repeats < repeatsThreshold;
    }

    /**
     * Returns the Camera Data in the default format (x, y, θ°) using Pose2d's toString().
     * Position, Velocity, and Confidence are separated on different lines.
     *
     * Position: (~, ~, ~°)
     * Velocity: (~, ~, ~°)
     * Confidence: ~
     * Unique: T/F
     */
    @NotNull
    public String toString() {
        String robotPos, robotVel, poseConf;
        boolean unique;

        robotPos = robotPosition.toString();
        robotVel = robotVelocity.toString();
        poseConf = poseConfidence.toString();
        unique = isUnique();

        return "Position: " + robotPos + "\n" +
               "Velocity: " + robotVel + "\n" +
               "Confidence: " + poseConf + "\n" +
               "Unique: " + unique;
    }

    /**
     * Returns the Camera Data in a labelled format. Position, Velocity, and Confidence
     * are separated on different lines.
     *
     * Position: (x: ~ in, y: ~ in, Heading: ~°)
     * Velocity: (Vx: ~ in/s, Vy: ~ in/s, Omega: ~ deg/s)
     * Confidence: ~
     * Repeats: ~
     */
    @NotNull
    public String toStringLabelled() {
        String robotPos, robotVel, poseConf;

        robotPos = String.format(Locale.ENGLISH,
                "(x: %.3f in, y: %.3f in, Heading: %.3f°)",
                robotPosition.getX(),
                robotPosition.getY(),
                Math.toDegrees(robotPosition.getHeading()));
        robotVel = String.format(Locale.ENGLISH,
                "(Vx: %.3f in/s, Vy: %.3f in/s, Omega: %.3f deg/s)",
                robotVelocity.getX(),
                robotVelocity.getY(),
                Math.toDegrees(robotVelocity.getHeading()));
        poseConf = poseConfidence.toString();

        return "Position: " + robotPos + "\n" +
               "Velocity: " + robotVel + "\n" +
               "Confidence: " + poseConf + "\n" +
               "Repeats: " + repeats;
    }

    /**
     * @return whether every value (except repeats) is the same
     */
    public boolean equals(@NotNull CameraData comparison) {
        return this.robotPosition == comparison.robotPosition &&
               this.robotVelocity == comparison.robotVelocity &&
               this.poseConfidence == comparison.poseConfidence;
    }
}

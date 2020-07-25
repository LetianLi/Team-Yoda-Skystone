package org.firstinspires.ftc.teamcode.drive.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.yoda_code.CameraData;
import org.firstinspires.ftc.teamcode.yoda_code.OdometryWheel;
import org.firstinspires.ftc.teamcode.yoda_code.T265CameraSystem;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 * Note: this could be optimized significantly with REV bulk reads
 */

@Config
public class T265CameraWithThreeTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    private Telemetry telemetry;
    public static T265CameraSystem slamra;

    private CameraData cameraData;
    private Pose2d poseEstimate;

    public static double TICKS_PER_REV = 2880;
    public static double WHEEL_RADIUS = 2.9/2.54; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 13 + 14.0/16.0; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 4 + 9.0/16.0; // in; offset of the lateral wheel // changed from 10/16

    public OdometryWheel leftEncoder, rightEncoder, frontEncoder;

    public T265CameraWithThreeTrackingWheelLocalizer(Telemetry telemetry, HardwareMap hardwareMap, FtcDashboard dashboard) {
        super(Arrays.asList(
                new Pose2d(1 + 3.0/16.0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(1 + 3.0/16.0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, -10.0/16.0, Math.toRadians(90)) // front, changed from -0.6
        ));

        leftEncoder = new OdometryWheel(hardwareMap, "leftEncoder", WHEEL_RADIUS, GEAR_RATIO, TICKS_PER_REV, true,
                new Pose2d(1 + 3.0/16.0, LATERAL_DISTANCE / 2, 0));

        rightEncoder = new OdometryWheel(hardwareMap, "rightEncoder", WHEEL_RADIUS, GEAR_RATIO, TICKS_PER_REV, false,
                new Pose2d(1 + 3.0/16.0, LATERAL_DISTANCE / 2, 0));

        frontEncoder = new OdometryWheel(hardwareMap, "frontEncoder", WHEEL_RADIUS, GEAR_RATIO, TICKS_PER_REV, true,
                new Pose2d(1 + 3.0/16.0, LATERAL_DISTANCE / 2, 0));

        this.telemetry = telemetry;
        slamra = new T265CameraSystem(telemetry, hardwareMap);
        slamra.init(true, new Pose2d(6.5, 5.5));
//        slamra.setUpOdometry(leftEncoder, frontEncoder);
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(frontEncoder.getCurrentPosition())
        );
    }

//    @Override
//    public void update() {
//        cameraData = slamra.getCameraData();
//        poseEstimate = cameraData.getRobotPosition();
//
//        telemetry.addLine(cameraData.toStringLabelled());
//    }
//
//    @NotNull
//    @Override
//    public Pose2d getPoseEstimate() {
//        return poseEstimate;
//    }
//
//    @Override
//    public void setPoseEstimate(@NotNull Pose2d newPoseEstimate) {
//        poseEstimate = newPoseEstimate;
//    }
}

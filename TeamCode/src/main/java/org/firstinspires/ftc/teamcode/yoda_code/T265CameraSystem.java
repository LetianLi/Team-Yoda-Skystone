package org.firstinspires.ftc.teamcode.yoda_code;

import android.content.Context;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ConversionUtil;

import java.util.function.Consumer;

public class T265CameraSystem {
    private static T265Camera T265CameraBase = null;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    private static CameraData cameraData = new CameraData();


    public T265CameraSystem(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    public void init(boolean reinitialize, Pose2d robotOffset, double odometryCovariance, String relocMapPath, Context appContext) {
        telemetry.addLine("Initializing Slamra...");
        telemetry.update();
        Log.d("T265CameraSystem", "Function Call: init() called");

        if (T265CameraBase == null || reinitialize) {
            Pose2d robotOffsetMeters = ConversionUtil.inchesToMeters(robotOffset);
            T265CameraBase = new T265Camera(ConversionUtil.toTransform2d(robotOffsetMeters), odometryCovariance, relocMapPath, appContext);
            telemetry.addLine("SLAMRA is Re-initialized");
            Log.d("T265CameraSystem", "Reinitialized - Reason: (Previously null - " + (T265CameraBase == null) + "), (Override Requested - " + reinitialize + ")");
        } else {
            telemetry.addLine("SLAMRA is already Initialized");
            Log.d("T265CameraSystem", "Already Initialized");
        }
        telemetry.update();
        Log.d("T265CameraSystem", "SLAMRA Name: " + T265CameraBase.toString());
    }

    public void init(boolean overridePrevious, Pose2d robotOffset, double odometryCovariance, Context appContext) {
        init(overridePrevious, robotOffset, odometryCovariance, "", appContext);
    }

    public void init(boolean overridePrevious) {
        init(overridePrevious, new Pose2d(), 0.1, hardwareMap.appContext);
    }

    public void init() {
        init(true);
    }

    public void start(Pose2d newPose, Consumer<T265Camera.CameraUpdate> poseConsumer) {
        Log.d("T265CameraSystem", "Function Call: start() called");
        setPose(newPose);
        T265CameraBase.start(poseConsumer);
    }

    public void start(Pose2d newPose) {
        start(newPose, (up) -> {
            if(up != null) {
                cameraData = new CameraData(up, cameraData);

                if (!cameraData.isUnique()) {
                    Log.d("T265CameraSystem", "Error: cameraData is not unique. This value has been repeated (" + cameraData.getRepeats() + ") times.");
                }
            }
        });
    }

    public void stop() {
        Log.d("T265CameraSystem", "Function Call: stop() called");
        T265CameraBase.stop();
    }

    public void setPose(Pose2d newPose) {
        Log.d("T265CameraSystem", "Function Call: setPose" + newPose.toString() + " called. (Display type inches)");

        Pose2d newPoseMeters = ConversionUtil.inchesToMeters(newPose);
        T265CameraBase.setPose(ConversionUtil.toLibPose2d(newPoseMeters));
    }

    public T265Camera.CameraUpdate getLastReceivedCameraUpdate() {
        Log.d("T265CameraSystem", "Function Call: getLastReceivedCameraUpdate() called");

        return T265CameraBase.getLastReceivedCameraUpdate();
    }

    public void exportRelocalizationMap(String path) {
        Log.d("T265CameraSystem", "Function Call: exportRelocalizationMap(" + path + ") called");

        T265CameraBase.exportRelocalizationMap(path);
    }

    public void free() {
        Log.d("T265CameraSystem", "Function Call: free() called");

        T265CameraBase.free();
    }

    public void sendOdometry(Vector2d translationalVelocity) {
        Log.d("T265CameraSystem", "Function Call: sendOdometry(...) called");

        Vector2d translationalVelocityMeters = ConversionUtil.inchesToMeters(translationalVelocity);
        T265CameraBase.sendOdometry(translationalVelocityMeters.getX(), translationalVelocityMeters.getY());
    }

    public CameraData getCameraData() {
        Log.d("T265CameraSystem", "Function Call: getCameraData() called"); // Expect Spam

        return cameraData;
    }
}

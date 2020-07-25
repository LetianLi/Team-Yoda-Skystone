package org.firstinspires.ftc.teamcode.yoda_code;

import android.content.Context;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ConversionUtil;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.List;
import java.util.function.Consumer;

public class T265CameraSystem {
    private static T265Camera T265CameraBase = null;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private OdometryWheel SupplierX = null;
    private OdometryWheel SupplierY = null;

    private static CameraData lastCameraData = new CameraData();
    private static CameraData cameraData = new CameraData();


    public T265CameraSystem(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    public void init(boolean reinitialize, Pose2d robotOffset, double odometryCovariance, String relocMapPath, Context appContext) {
        telemetry.addLine("Initializing Slamra...");
        telemetry.update();
        Log.i("T265CameraSystem", "Function Call: init() called");

        if (T265CameraBase == null || reinitialize) {
            Pose2d robotOffsetMeters = ConversionUtil.inchesToMeters(robotOffset);
            T265CameraBase = new T265Camera(ConversionUtil.toTransform2d(robotOffsetMeters), odometryCovariance, relocMapPath, appContext);
            telemetry.addLine("SLAMRA is Re-initialized");
            Log.i("T265CameraSystem", "Reinitialized - Reason: (Previously null - " + (T265CameraBase == null) + "), (Override Requested - " + reinitialize + ")");
        } else {
            telemetry.addLine("SLAMRA is already Initialized");
            Log.i("T265CameraSystem", "Already Initialized");
        }
        telemetry.update();
        Log.i("T265CameraSystem", "SLAMRA Name: " + T265CameraBase.toString());
    }

    public void init(boolean overridePrevious, Pose2d robotOffset, double odometryCovariance, Context appContext) {
        init(overridePrevious, robotOffset, odometryCovariance, "", appContext);
    }

    public void init(boolean overridePrevious, Pose2d robotOffset) {
        init(overridePrevious, robotOffset, 0.1, hardwareMap.appContext);
    }

    public void init() {
        init(true, new Pose2d());
    }

    public void start(Pose2d newPose, Consumer<T265Camera.CameraUpdate> poseConsumer) {
        Log.i("T265CameraSystem", "Function Call: start() called");
        setPose(newPose);
        T265CameraBase.start(poseConsumer);
    }

    public void start(Pose2d newPose) {
        start(newPose, (cameraOutput) -> {
            if(cameraOutput != null) {
                lastCameraData = cameraData.clone();
                cameraData = new CameraData(cameraOutput, lastCameraData);

                if (!cameraData.isUnique()) {
                    Log.d("T265CameraSystem", "Error: cameraData is not unique. This value has been repeated (" + cameraData.getRepeats() + ") times.");
                }

                if (SupplierX != null && SupplierY != null){
                    sendOdometry(getWheelOdometryVelocity());
                }
            }
        });
    }

    public void stop() {
        Log.i("T265CameraSystem", "Function Call: stop() called");
        T265CameraBase.stop();
    }

    public void setPose(Pose2d newPose) {
        Log.i("T265CameraSystem", "Function Call: setPose" + newPose.toString() + " called. (Display type inches)");

        Pose2d newPoseMeters = ConversionUtil.inchesToMeters(newPose);
        T265CameraBase.setPose(ConversionUtil.toLibPose2d(newPoseMeters));
    }

    public T265Camera.CameraUpdate getLastReceivedCameraUpdate() {
        Log.i("T265CameraSystem", "Function Call: getLastReceivedCameraUpdate() called");

        return T265CameraBase.getLastReceivedCameraUpdate();
    }

    public void exportRelocalizationMap(String path) {
        Log.i("T265CameraSystem", "Function Call: exportRelocalizationMap(" + path + ") called");

        T265CameraBase.exportRelocalizationMap(path);
    }

    public void free() {
        Log.i("T265CameraSystem", "Function Call: free() called");

        T265CameraBase.free();
    }

    public void sendOdometry(Vector2d translationalVelocity) {
        Log.i("T265CameraSystem", "Function Call: sendOdometry(...) called");

        Vector2d translationalVelocityMeters = ConversionUtil.inchesToMeters(translationalVelocity);
        T265CameraBase.sendOdometry(translationalVelocityMeters.getX(), translationalVelocityMeters.getY());
    }

    public CameraData getCameraData() {
//        Log.i("T265CameraSystem", "Function Call: getCameraData() called"); // Expect Spam

        return cameraData;
    }

    public Vector2d getWheelOdometryVelocity() {
        Log.i("T265CameraSystem", "Function Call: getWheelOdometryVelocity() called");
        return new Vector2d(SupplierX.getVelocityInches(), SupplierY.getVelocityInches());
    }

    public void setUpOdometry(OdometryWheel supplierX, OdometryWheel supplierY) {
        Log.i("T265CameraSystem", "Function Call: setUpOdometry() called");
        this.SupplierX = supplierX;
        this.SupplierY = supplierY;
    }
}

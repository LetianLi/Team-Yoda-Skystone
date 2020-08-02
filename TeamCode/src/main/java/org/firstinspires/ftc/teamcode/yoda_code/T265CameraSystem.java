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

import static org.firstinspires.ftc.teamcode.yoda_code.YodaMecanumDrive.localizer;
import static org.firstinspires.ftc.teamcode.yoda_code.YodaMecanumDrive.slamra;

public class T265CameraSystem {
    private static T265Camera T265CameraBase = null;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    private static CameraData cameraData = new CameraData();


    public T265CameraSystem(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    /**
     *
     * @param robotOffset The position of the camera in relation to the robot's (0, 0, 0°)
     * @param odometryCovariance How much fusion (0-1)
     * @param relocMapPath path (including filename) to a relocalization map to load.
     */
    public void init(Pose2d robotOffset, double odometryCovariance, String relocMapPath) {
        telemetry.addLine("Initializing Slamra...");
        telemetry.update();
        Log.i("T265CameraSystem", "Function Call: init() called");

        if (T265CameraBase != null) {
            Log.i("T265CameraSystem", "State: Already Initialized - Reinitializing");
//            T265CameraBase.free(); // This causes nativeCameraObjectPointer Exception
        }

        // Convert Pose2d into one with yaw (multiply heading by -1) and as meters
        robotOffset = new Pose2d(robotOffset.vec(), -robotOffset.getHeading());
        Pose2d robotOffsetMeters = ConversionUtil.inchesToMeters(robotOffset);

        T265CameraBase = new T265Camera(ConversionUtil.toTransform2d(robotOffsetMeters), odometryCovariance, relocMapPath, hardwareMap.appContext);
        telemetry.addLine("SLAMRA is Re-initialized");
        Log.i("T265CameraSystem", "State: Reinitialized");

        telemetry.update();
        Log.i("T265CameraSystem", "State: SLAMRA Name: " + T265CameraBase.toString());
    }

    public void init(Pose2d robotOffset, double odometryCovariance) {
        init(robotOffset, odometryCovariance, "");
    }

    public void init(Pose2d robotOffset) {
        init(robotOffset, 0.1);
    }

    public void init() {
        init(new Pose2d());
    }

    /**
     *
     * @param newPose The pose the camera should be zeroed to. (-1 to yaw not yet applied)
     * @param poseConsumer A method to be called every time we receive a pose from
     *                     <i>from a different thread</i>! You must synchronize
     *                     memory access across threads!
     *                     Received poses are in meters.
     */
    public void start(Pose2d newPose, Consumer<T265Camera.CameraUpdate> poseConsumer) {
        Log.i("T265CameraSystem", "Function Call: start() called");
        setPose(newPose);
        T265CameraBase.start(poseConsumer);
    }

    public void start(Pose2d newPose) {
        start(newPose, (cameraOutput) -> {
            if(cameraOutput != null) {
//                localizer.updatePoseDelta();
                slamra.sendOdometry(localizer.getPoseVelocity());
                cameraData = new CameraData(cameraOutput, cameraData);

                if (!cameraData.isUnique()) {
                    Log.d("T265CameraSystem", "Error: cameraData is not unique. This value has been repeated (" + cameraData.getRepeats() + ") times.");
                }

                Log.i("T265CameraSystem", "Function Call: update() called");
            }
        });
    }

    public void stop() {
        Log.i("T265CameraSystem", "Function Call: stop() called");
        T265CameraBase.stop();
    }

    public void setPose(Pose2d newPose) {
        Log.i("T265CameraSystem", "Function Call: setPose" + newPose.toString() + " called");

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

    public void sendOdometry(Pose2d translationalVelocity) {
        sendOdometry(translationalVelocity.vec());
    }

    public CameraData getCameraData() {
//        Log.i("T265CameraSystem", "Function Call: getCameraData() called"); // Expect Spam

        return cameraData;
    }
}

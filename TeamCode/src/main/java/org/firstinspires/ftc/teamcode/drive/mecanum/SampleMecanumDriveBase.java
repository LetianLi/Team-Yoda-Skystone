package org.firstinspires.ftc.teamcode.drive.mecanum;

import android.util.Log;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * Base class with shared functionality for sample mecanum drives. All hardware-specific details are
 * handled in subclasses.
 */
public abstract class SampleMecanumDriveBase extends MecanumDrive {
    protected LinearOpMode opMode;
    public static PIDCoefficients AXIAL_PID = new PIDCoefficients(1.75, 0, 0.07); // X    9, 0, 0
    public static PIDCoefficients LATERAL_PID = new PIDCoefficients(1, 0, 0); // Y 10, 0, 0
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(3.2, 0, 0); // Heading 7, 0, 0


    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    private FtcDashboard dashboard;
    private NanoClock clock;

    private Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private DriveConstraints constraints;
    private TrajectoryFollower follower;

    private List<Double> lastWheelPositions;
    private double lastTimestamp;
    public ElapsedTime global_timer;
    public ElapsedTime latency_timer;
    public String last_tag_for_logging;

    public SampleMecanumDriveBase() {
        super(kV, kA, kStatic, TRACK_WIDTH);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(AXIAL_PID, LATERAL_PID, HEADING_PID);
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getPoseEstimate(), constraints);
    }

    public void turn(double angle) {
        double heading = getPoseEstimate().getHeading();
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
        );
        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    public void turnSync(double angle) {
        log("Turn - Start Position: " + getPoseEstimate().toString());
        turn(angle);
        latency_timer.reset();
        waitForIdle();
        log("Turn - End Position: " + getPoseEstimate().toString());
    }

    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectorySync(Trajectory trajectory) {
        log("Trajectory - Start Position: " + getPoseEstimate().toString());
                //+ String.format("--- IMU:%.3f", Math.toDegrees(getRawExternalHeading())));
        followTrajectory(trajectory);
        latency_timer.reset();
        waitForIdle();
        log("Trajectory - End Position: " + getPoseEstimate().toString());
                //+ String.format("--- IMU:%.3f", Math.toDegrees(getRawExternalHeading())));
    }

    public void followTrajectorySync(Trajectory trajectory, double xTolerance, double yTolerance) {
        log("Trajectory - Start Position: " + getPoseEstimate().toString());
        //+ String.format("--- IMU:%.3f", Math.toDegrees(getRawExternalHeading())));
        followTrajectory(trajectory);
        latency_timer.reset();
        while (!Thread.currentThread().isInterrupted() && isBusy() && !opMode.isStopRequested()) {
            update();
            if (xTolerance != 0) { if (getLastError().getX() > xTolerance) opMode.requestOpModeStop();}
            if (yTolerance != 0) { if (getLastError().getY() > yTolerance) opMode.requestOpModeStop();}
        }
        log("Trajectory - End Position: " + getPoseEstimate().toString());
        //+ String.format("--- IMU:%.3f", Math.toDegrees(getRawExternalHeading())));
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void update() {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", Math.toDegrees(currentPose.getHeading()));

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError", Math.toDegrees(lastError.getHeading()));
        packet.put("Latency", latency_timer.milliseconds());
/*
        setLogTag("update");
        log("Latency:" + (int)(latency_timer.milliseconds())
                + ",x:"+currentPose.getX()
                +",y:"+currentPose.getY()
                + ",heading:"+Math.toDegrees(currentPose.getHeading())
                + ",xError:"+lastError.getX()
                + ",yError:"+lastError.getY()
                + ",headingError:"+ Math.toDegrees(lastError.getHeading())
        );

 */

        latency_timer.reset();


        switch (mode) {
            case IDLE:
                // do nothing

                fieldOverlay.setStroke("#F44336");
                DashboardUtil.drawRobot(fieldOverlay, currentPose);

                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                double correction = turnController.update(currentPose.getHeading(), targetOmega);

                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }
                fieldOverlay.setStroke("#F44336");
                DashboardUtil.drawRobot(fieldOverlay, currentPose);

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose));

                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("4CAF50");
                if (trajectory.getPath().length() > 100) {
                    log("trajectory length:" + trajectory.getPath().length());
                }
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());

                fieldOverlay.setStroke("#4F61C5");
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#F44336");
                DashboardUtil.drawRobot(fieldOverlay, currentPose); // The current robot location

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        dashboard.sendTelemetryPacket(packet);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy() && !opMode.isStopRequested()) {
            update();
        }
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    public List<Double> getWheelVelocities() {
        List<Double> positions = getWheelPositions();
        double currentTimestamp = clock.seconds();

        List<Double> velocities = new ArrayList<>(positions.size());;
        if (lastWheelPositions != null) {
            double dt = currentTimestamp - lastTimestamp;
            for (int i = 0; i < positions.size(); i++) {
                velocities.add((positions.get(i) - lastWheelPositions.get(i)) / dt);
            }
        } else {
            for (int i = 0; i < positions.size(); i++) {
                velocities.add(0.0);
            }
        }

        lastTimestamp = currentTimestamp;
        lastWheelPositions = positions;

        return velocities;
    }

    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public abstract PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode);

    public abstract void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients);

    public void log(String message) {
        Log.i("Yoda","(" + global_timer.time(TimeUnit.MILLISECONDS) + "ms)" + last_tag_for_logging + " | " + message);
    }

    public void setLogTag(String tag) {
        last_tag_for_logging = tag;
    }

    public double getCalculatedY(double startingY) {
        // Overriden in YodaMecanumDrive
        return getPoseEstimate().getY();
    }
}

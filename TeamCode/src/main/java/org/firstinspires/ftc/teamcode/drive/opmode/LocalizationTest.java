package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.yoda_code.YodaMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
//@Disabled
@Config
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;
    public static double DISTANCE = 10;

    @Override
    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        boolean braking = false;

        YodaMecanumDrive drive = new YodaMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
        drive.resetServos();
        waitForStart();

        while (!isStopRequested()) {
            if (gamepad1.a) {
                drive.setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                braking = true;
            }
            else if (gamepad1.b) {
                drive.setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                braking = false;
            }
            if (gamepad1.x) drive.setPoseEstimate(new Pose2d());

            double bumperTurn = 0;

            if (gamepad1.dpad_up) drive.forward(DISTANCE);
            if (gamepad1.dpad_down) drive.back(DISTANCE);
            if (gamepad1.dpad_right) drive.strafeRight(DISTANCE);
            if (gamepad1.dpad_left) drive.strafeLeft(DISTANCE);
            if (gamepad1.right_bumper) bumperTurn += 0.1;
            if (gamepad1.left_bumper) bumperTurn -= 0.1;

            Pose2d baseVel = new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x - bumperTurn
            );

            Pose2d vel;
            if (Math.abs(baseVel.getX()) + Math.abs(baseVel.getY()) + Math.abs(baseVel.getHeading()) > 1) {
                // re-normalize the powers according to the weights
                double denom = VX_WEIGHT * Math.abs(baseVel.getX())
                    + VY_WEIGHT * Math.abs(baseVel.getY())
                    + OMEGA_WEIGHT * Math.abs(baseVel.getHeading());
                vel = new Pose2d(
                    VX_WEIGHT * baseVel.getX(),
                    VY_WEIGHT * baseVel.getY(),
                    OMEGA_WEIGHT * baseVel.getHeading()
                ).div(denom);
            } else {
                vel = baseVel;
            }

            drive.setDrivePower(vel);

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
//            telemetry.addData("Encoder values, lf, lr, rr, rf", drive.getWheelPositions());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("IMU", Math.toDegrees(drive.getRawExternalHeading()));
            telemetry.addData("Braking", braking);
            telemetry.update();
        }
    }
}

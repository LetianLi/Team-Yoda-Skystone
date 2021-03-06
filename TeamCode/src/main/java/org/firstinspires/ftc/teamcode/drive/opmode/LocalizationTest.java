package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
    private double VX_WEIGHT = 1;
    private double VY_WEIGHT = 1;
    private double OMEGA_WEIGHT = 1;
    public static double DISTANCE = 70;
    public static double STARTINGX = 0;
    public static double STARTINGY = 0;
    public static double STARTINGHEADINGDEG = 0;
    public static boolean TESTINGMODE = false;
    public static boolean BRAKING = false;

    private YodaMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new YodaMecanumDrive(telemetry, hardwareMap);
//        drive.setPoseEstimate(new Pose2d(STARTINGX, STARTINGY, Math.toRadians(STARTINGHEADINGDEG)));
//        drive.resetLeftSensorToWall(STARTINGY, 0);
        drive.resetInitServos();
        waitForStart();
        drive.slamra.start(new Pose2d());

        while (!isStopRequested()) {
            if (gamepad1.a) {
                drive.setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                BRAKING = true;
            }
            else if (gamepad1.b) {
                drive.setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                BRAKING = false;
            }
//            if (gamepad1.x) {
//                drive.setPoseEstimate(new Pose2d(0, 0, 0));
//                drive.resetLeftSensorToWall(0, 0);
//            }

            double bumperTurn = 0;
            double extraX = 0;
            double extraY = 0;

            if (TESTINGMODE) {
                if (!drive.isBusy()) {
                    if (gamepad1.dpad_up) drive.followTrajectory(drive.trajectoryBuilder().forward(DISTANCE).build());
                    if (gamepad1.dpad_down) drive.followTrajectory(drive.trajectoryBuilder().back(DISTANCE).build());
                    if (gamepad1.dpad_right) drive.followTrajectory(drive.trajectoryBuilder().strafeRight(DISTANCE).build());
                    if (gamepad1.dpad_left) drive.followTrajectory(drive.trajectoryBuilder().strafeLeft(DISTANCE).build());
                    if (gamepad1.y) {
                        drive.followTrajectory(drive.trajectoryBuilder().strafeTo(new Vector2d(0, 0)).build());
                    }
                }
            }
            else {
                if (gamepad1.dpad_up) extraY += 0.1;
                if (gamepad1.dpad_down) extraY -= 0.1;
                if (gamepad1.dpad_right) extraX += 0.1;
                if (gamepad1.dpad_left) extraX -= 0.1;
            }
            if (gamepad1.right_bumper) bumperTurn += 0.1;
            if (gamepad1.left_bumper) bumperTurn -= 0.1;

            if (!drive.isBusy()) {
                Pose2d baseVel = new Pose2d(
                        -gamepad1.left_stick_y + extraY,
                        -gamepad1.left_stick_x - extraX,
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
            }

            drive.update();

//            Pose2d poseEstimate = drive.getPoseEstimate();
//            double calcY = drive.getCalculatedY(STARTINGY);
//            double dist = drive.getLeftDistance();
//            telemetry.addData("Encoder values, lf, lr, rr, rf", drive.getWheelPositions());
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("Calculated y", calcY);
//            telemetry.addData("Y difference", calcY - poseEstimate.getY());
//            telemetry.addData("Left Distance", dist);
//            telemetry.addData("Calc - Dist", calcY - dist);
//            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
//            telemetry.addData("IMU", Math.toDegrees(drive.getRawExternalHeading()));
//            telemetry.addData("Braking", BRAKING);
//            telemetry.addData("Left Encoder", encoderTicksToInches(drive.leftEncoder.getCurrentPosition()));
//            telemetry.addData("Right Encoder", encoderTicksToInches(drive.rightEncoder.getCurrentPosition()));
//            telemetry.addData("Front Encoder", encoderTicksToInches(drive.frontEncoder.getCurrentPosition()));
            telemetry.update();
        }
        drive.slamra.stop();
        drive.slamra.free();
    }
}

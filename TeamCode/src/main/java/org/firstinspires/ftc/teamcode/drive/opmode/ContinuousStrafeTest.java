package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.yoda_code.YodaMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
//@Disabled
@Config
@TeleOp(group = "drive")
public class ContinuousStrafeTest extends LinearOpMode {
    public static int repetitions = 10;
    public static double distance = 60;
    private YodaMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new YodaMecanumDrive(hardwareMap);
        Pose2d startPose, endPose;
        Vector2d targetPose;
        waitForStart();

        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d());

        for (int i = 1; i <= repetitions; i++) {
            targetPose = new Vector2d(0, drive.getPoseEstimate().getY() + distance);
            startPose = drive.getPoseEstimate();
            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .strafeTo(targetPose)
                    .build());
//            waitForIdle();
            endPose = drive.getPoseEstimate();
            logTrajectory(i + "/" + repetitions + " - Strafe Left " + distance, startPose, endPose, targetPose);

            targetPose = new Vector2d(0, drive.getPoseEstimate().getY() - distance);
            startPose = drive.getPoseEstimate();
            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .strafeTo(targetPose)
                    .build());
//            waitForIdle();
            endPose = drive.getPoseEstimate();
            logTrajectory(i + "/" + repetitions + " - Strafe Right " + distance, startPose, endPose, targetPose);
        }
        drive.setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        telemetryIdle();
    }

    private void telemetryIdle() {
        while (!isStopRequested()) {
            drive.update();
            telemetry.addData("IMU", Math.toDegrees(drive.getRawExternalHeading()));
            telemetry.addData("Heading", Math.toDegrees(drive.getPoseEstimate().getHeading()));
            telemetry.addData("X", drive.getPoseEstimate().getX());
            telemetry.addData("Y", drive.getPoseEstimate().getY());
            telemetry.update();
        }
    }
    private void logTrajectory(String context, Pose2d startPose, Pose2d endPose, Vector2d targetPose) {
        drive.log(context);
        drive.log("Target : " + startPose.toString() + " --> " + targetPose.toString());
        drive.log("Reality: " + startPose.toString() + " --> " + endPose.toString());
        drive.log("Error  : " + endPose.vec().minus(targetPose));
        drive.log("Change : " + endPose.minus(startPose).toString());
        drive.log("");
    }
}

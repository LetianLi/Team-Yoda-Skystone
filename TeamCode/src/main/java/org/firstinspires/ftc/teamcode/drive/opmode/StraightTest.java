package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.yoda_code.YodaMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Disabled
@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 60;

    @Override
    public void runOpMode() throws InterruptedException {
        YodaMecanumDrive drive = new YodaMecanumDrive(hardwareMap);
        Trajectory trajectory;
        if (DISTANCE < 0) {
            trajectory = drive.trajectoryBuilder()
                    .back(Math.abs(DISTANCE))
                    .build();
        }
        else {
            trajectory = drive.trajectoryBuilder()
                    .forward(DISTANCE)
                    .build();
        }

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);
        while (!isStopRequested()) {
            drive.update();
            telemetry.addData("IMU", Math.toDegrees(drive.getRawExternalHeading()));
            telemetry.addData("Heading", Math.toDegrees(drive.getPoseEstimate().getHeading()));
            telemetry.addData("X", drive.getPoseEstimate().getX());
            telemetry.addData("Y", drive.getPoseEstimate().getY());
            telemetry.addData("Left Encoder", drive.leftEncoder.getCurrentPosition());
            telemetry.addData("Right Encoder", drive.rightEncoder.getCurrentPosition());
            telemetry.addData("Front Encoder", drive.frontEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}

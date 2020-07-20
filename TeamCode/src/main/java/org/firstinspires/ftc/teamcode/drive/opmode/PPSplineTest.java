package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.yoda_code.YodaMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(group = "drive")
public class PPSplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        YodaMecanumDrive drive = new YodaMecanumDrive(telemetry, hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.followPathSync(
                drive.pathBuilder()
                        .splineTo(new Pose2d(30, 30, 0))
                        .build()
        );

        sleep(2000);

        drive.followPathSync(
                drive.pathBuilder()
                        .reverse()
                        .splineTo(new Pose2d(0, 0, 0))
                        .build()
        );
    }
}

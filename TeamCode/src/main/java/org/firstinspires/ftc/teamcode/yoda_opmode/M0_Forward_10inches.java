package org.firstinspires.ftc.teamcode.yoda_opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.yoda_code.YodaMecanumDrive;

//@Disabled
@Config
@Autonomous(group = "auto", name = "M0 forward 10 inches")
public class M0_Forward_10inches extends LinearOpMode {
    private YodaMecanumDrive drive;
    private static int delay_seconds = 20;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new YodaMecanumDrive(telemetry, hardwareMap);
        drive.setOpMode(this);
        drive.resetTimer();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Initialized. Wait to start");
            telemetry.update();

            idle();
            sleep(50);
        }

        if (isStopRequested()) return;

        double delay = delay_seconds * 1000;
        double startTime = drive.global_timer.milliseconds();

        while (drive.global_timer.milliseconds() <= startTime + delay) {
            telemetry.addData("ms left", (int) ((startTime + delay) - drive.global_timer.milliseconds()));
            telemetry.update();
        }

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(10)
                .build());
    }
}

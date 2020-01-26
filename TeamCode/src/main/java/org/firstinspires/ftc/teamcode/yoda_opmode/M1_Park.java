package org.firstinspires.ftc.teamcode.yoda_opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.yoda_code.AutonomousBase;
import org.firstinspires.ftc.teamcode.yoda_code.YodaMecanumDrive;

//@Disabled
@Config
@Autonomous(group = "auto", name = "M1 Park")
public class M1_Park extends LinearOpMode {
    private YodaMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new YodaMecanumDrive(hardwareMap);
        drive.setOpMode(this);
        drive.resetTimer();
//        double inches = 12;
        long delay = 25;
        boolean setup = true;
        boolean pressedUpDown = false;
        boolean pressedLeftRight = false;
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        drive.setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while (!isStarted() && !isStopRequested()) {
            if ((gamepad1.dpad_up || gamepad2.dpad_up) && !pressedUpDown) {
                delay += 5;
                pressedUpDown = true;
            }

            if ((gamepad1.dpad_down || gamepad2.dpad_down) && !pressedUpDown) {
                delay -= 5;
                pressedUpDown = true;
            }
            if (!(gamepad1.dpad_up || gamepad2.dpad_up || gamepad1.dpad_down || gamepad2.dpad_down) && pressedUpDown) {
                pressedUpDown = false;
            }

//          ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

            if ((gamepad1.dpad_right || gamepad2.dpad_right) && !pressedLeftRight) {
                delay += 1;
                pressedLeftRight = true;
            }

            if ((gamepad1.dpad_left || gamepad2.dpad_left) && !pressedLeftRight) {
                delay -= 1;
                pressedLeftRight = true;
            }
            if (!(gamepad1.dpad_right || gamepad2.dpad_right || gamepad1.dpad_left || gamepad2.dpad_left) && pressedLeftRight) {
                pressedLeftRight = false;
            }

            delay = drive.minMax(delay, 0, 30);

            if (gamepad1.a || gamepad2.a) setup = true;
            else if (gamepad1.b || gamepad2.b) setup = false;

            if (gamepad1.x || gamepad2.x) drive.setPoseEstimate(new Pose2d(0, 0, 0));

            drive.updatePoseEstimate();
            telemetry.addData("Status", "Initialized. Wait to start");
            telemetry.addData("Delay (seconds)", delay);
            telemetry.addData("Position", drive.getPoseEstimate().toString());
            telemetry.addData("Setup", setup);
            telemetry.update();

            idle();
            sleep(50);
        }

        if (isStopRequested()) return;
        delay *= 1000;
        double startTime = drive.global_timer.milliseconds();

        while (drive.global_timer.milliseconds() <= startTime + delay) {
            telemetry.addData("ms left", (int) ((startTime + delay) - drive.global_timer.milliseconds()));
            telemetry.update();
        }

        if (setup) drive.horizontalExtender.setPosition(1);

        drive.updatePoseEstimate();
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .lineTo(new Vector2d(0, 0), new ConstantInterpolator(0))
                .build());

        sleep(5000);
    }
}

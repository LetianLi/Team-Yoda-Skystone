package org.firstinspires.ftc.teamcode.yoda_opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.yoda_code.YodaMecanumDrive;

@Config
@TeleOp(group = "test")
@Disabled
public class MotorTestingV2 extends LinearOpMode {
    private YodaMecanumDrive drive;
    public static boolean negative = false;
    public static double speed = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new YodaMecanumDrive(hardwareMap);
        drive.setOpMode(this);
        drive.resetTimer();

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clear();
        waitForStart();

        if (isStopRequested()) return;
        drive.setMotorNoEncoder();

        drive.resetInitServos(); // open, set power so that it does not go down

        while (!isStopRequested()) {
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;

            double leftFront = 0;
            double leftRear = 0;
            double rightRear = 0;
            double rightFront = 0;

            if (gamepad1.a) negative = false;
            else if (gamepad1.b) negative = true;

            double negativeMultiplier = negative ? -1 : 1;

            if (left && up) leftFront = speed * negativeMultiplier;
            if (left && down) leftRear = speed * negativeMultiplier;
            if (right && down) rightRear = speed * negativeMultiplier;
            if (right && up) rightFront = speed * negativeMultiplier;

            drive.setMotorPowers(leftFront, leftRear, rightRear, rightFront);

            telemetry.addData("Powers (lf, lr, rr, rf)", "[" + leftFront + ", " + leftRear + ", " + rightRear + ", " + rightFront + "]");
            telemetry.addData("Positions (lf, lr, rr, rf)", drive.getWheelPositions());
            telemetry.addData("Velocities (lf, lr, rr, rf)", drive.getWheelVelocities());
            telemetry.update();
        }
    }
}


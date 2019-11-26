package org.firstinspires.ftc.teamcode.yodacode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "drive")
public class ManualDrive extends LinearOpMode {
    private YodaMecanumDrive drive;
    private boolean[] pressed = new boolean[5];

    private double speed_multiplier = 1;
    private boolean isSlowMode = false;
    private double GLOBAL_SPEED_MULTIPLIER = 1.4; //somehow it only read 0.7 power, use this to get to 1
    private double SLOW_MODE_MULTIPLIER = 0.5;
    private double TURNING_SPEED = 1;
    private double DPAD_SPEED_MULTIPLIER = 0.3;

    private String driveMode = "Mecanum Drive";
    private String foundationMoverMode = "Stored";
    private String skystoneGrabberMode = "|| \n|";
    private String capstoneArmMode = "Stored";
    private double horizontalPosition = 0;
    private double previousHorizontalPos = -1;
    private final double insideHorizontal = 0.13;
    private final double outsideHorizontal = 0.33; // Actually 0.37

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new YodaMecanumDrive(hardwareMap);
        drive.setTelemetry(telemetry);

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clear();
        gamepad1.setJoystickDeadzone(0.1f);
        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {

            moveRobot();
            intake();
            controlExtenders();
            moveFoundationServo();
            controlSkystoneGrabbers();

            telemetry.addData("Drive Mode", driveMode);
            telemetry.addData("Speed co-efficients", "turn *%.2f, entire *%.2f", TURNING_SPEED, speed_multiplier);
            telemetry.addData("Encoder values, lf, lr, rr, rf", drive.getWheelPositions());
            telemetry.addData("Foundation Mover", foundationMoverMode);
            telemetry.addData("Capstone Arm", capstoneArmMode);
            telemetry.addData("Skystone Grabber", "\n" + skystoneGrabberMode + "\n|");
            telemetry.addData("Horizontal Pos", drive.horizontalExtender.getPosition());
            telemetry.update();
        }
    }

    private void moveFoundationServo() {
        if (gamepad1.x && !pressed[0]) {
            pressed[0] = true;
            if (foundationMoverMode == "Stored") foundationMoverMode = "Grab";
            else if (foundationMoverMode == "Grab") foundationMoverMode = "Stored";
        } else if (pressed[0] && !gamepad1.x) {
            pressed[0] = false;
        }

        if (foundationMoverMode == "Stored") {
            drive.foundationMoverLeft.setPosition(0.4);
            drive.foundationMoverRight.setPosition(0.3);
        } else if (foundationMoverMode == "Grab") {
            drive.foundationMoverLeft.setPosition(0.7);
            drive.foundationMoverRight.setPosition(0.6);
        }
    }

    private void moveRobot() {
        if (gamepad1.a && !pressed[1]) {
            isSlowMode = ! isSlowMode;
            pressed[1] = true;
        } else if (!gamepad1.a && pressed[1]) {
            pressed[1] = false;
        }

        double leftFrontPower = 0;
        double rightFrontPower = 0;
        double leftRearPower = 0;
        double rightRearPower = 0;

        double input_x = gamepad1.left_stick_x;
        double input_y = -gamepad1.left_stick_y;
        double input_turning = gamepad1.right_stick_x;

        boolean dpad_pressed = false;
        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            input_y = DPAD_SPEED_MULTIPLIER;
            input_x = 0;
            dpad_pressed = true;
        } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
            input_y = -DPAD_SPEED_MULTIPLIER;
            input_x = 0;
            dpad_pressed = true;
        }

        if(gamepad1.dpad_right || gamepad2.dpad_right) {
            input_x = DPAD_SPEED_MULTIPLIER;
            input_y = 0;
            dpad_pressed = true;
        } else if (gamepad1.dpad_left || gamepad2.dpad_left) {
            input_x = -DPAD_SPEED_MULTIPLIER;
            input_y = 0;
            dpad_pressed = true;
        }

        if (gamepad1.left_bumper) {
            input_turning = DPAD_SPEED_MULTIPLIER;
            dpad_pressed = true;
        } else if (gamepad1.right_bumper) {
            input_turning = -DPAD_SPEED_MULTIPLIER;
            dpad_pressed = true;
        }

        if (dpad_pressed) {
            speed_multiplier = 1;
        } else if (isSlowMode) {
            speed_multiplier = SLOW_MODE_MULTIPLIER;
        }
        speed_multiplier = speed_multiplier * GLOBAL_SPEED_MULTIPLIER;

        double r = Math.hypot(input_x, input_y);
        double robotAngle = Math.atan2(input_y, input_x) - Math.PI / 4;

        leftFrontPower = (r * Math.cos(robotAngle) + TURNING_SPEED * input_turning) * speed_multiplier;
        rightFrontPower = (r * Math.sin(robotAngle) - TURNING_SPEED * input_turning) * speed_multiplier;
        leftRearPower = (r * Math.sin(robotAngle) + TURNING_SPEED * input_turning) * speed_multiplier;
        rightRearPower = (r * Math.cos(robotAngle) - TURNING_SPEED * input_turning) * speed_multiplier;


        drive.setMotorPowers(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);
        telemetry.addData("Wheel Powers", "lf %.2f, lr %.2f, rr %.2f, rf %.2f", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
    }

    private void intake() {
        if (gamepad2.a) drive.intakeGrabber.setPosition(0);
        if (gamepad2.b) drive.intakeGrabber.setPosition(0.4);

        if (gamepad2.x && !pressed[3]) {
            if (capstoneArmMode == "Stored") capstoneArmMode = "Ready";
            else if (capstoneArmMode == "Ready") capstoneArmMode = "Dropping";
            else if (capstoneArmMode == "Dropping") capstoneArmMode = "Dropped";
            else if (capstoneArmMode == "Dropped") capstoneArmMode = "Stored";
            pressed[3] = true;
        } else if (!gamepad2.x && pressed[3]) {
            pressed[3] = false;
        }
        if (capstoneArmMode == "Dropping" && drive.capstoneArm.getPosition() >= 0.988)
            capstoneArmMode = "Dropped";

        if (capstoneArmMode == "Stored") {
            drive.capstoneArm.setPosition(0.31);
        } else if (capstoneArmMode == "Ready") {
            drive.capstoneArm.setPosition(0.92);
            drive.intakeGrabber.setPosition(0.5);
            drive.horizontalExtender.setPosition(insideHorizontal);
        } else if (capstoneArmMode == "Dropping") {
            drive.capstoneArm.setPosition(drive.capstoneArm.getPosition() + 0.002);
        } else if (capstoneArmMode == "Dropped") {
            drive.capstoneArm.setPosition(0.99);
        }

    }

    private void controlExtenders() {
        double verticalPower = -gamepad2.right_stick_y;
        if (verticalPower == 0 && drive.verticalExtender.getCurrentPosition() > 20) {
            //verticalPower = 0.05;
        }
        drive.verticalExtender.setPower(verticalPower);
        horizontalPosition = Math.min(Math.max(horizontalPosition - gamepad2.left_stick_y / 300, insideHorizontal), outsideHorizontal);
        if (horizontalPosition != previousHorizontalPos) drive.horizontalExtender.setPosition(horizontalPosition);
        previousHorizontalPos = horizontalPosition;
    }

    private void controlSkystoneGrabbers() {
        if (gamepad1.y && !pressed[2]) {
            if (skystoneGrabberMode == "|| \n|") skystoneGrabberMode = "|-- \n|";
            else if (skystoneGrabberMode == "|-- \n|") skystoneGrabberMode = "\n----|";
            else if (skystoneGrabberMode == "\n----|") skystoneGrabberMode = "\n--[]";
            else if (skystoneGrabberMode == "\n--[]") skystoneGrabberMode = "|[] \n|";
            else if (skystoneGrabberMode == "|[] \n|") skystoneGrabberMode = "\n--[_]";
            else if (skystoneGrabberMode == "\n--[_]") skystoneGrabberMode = "\n--_--";
            else if (skystoneGrabberMode == "\n--_--") skystoneGrabberMode = "|| \n|";
            pressed[2] = true;
        } else if (!gamepad1.y && pressed[2]) {
            pressed[2] = false;
        }

        double skystoneArmDownPosition = 0.33;
        if (skystoneGrabberMode == "|| \n|") {
            drive.skystoneArmFront.setPosition(0);
            drive.skystoneGrabberFront.setPosition(0);
            drive.skystoneArmBack.setPosition(0);
            drive.skystoneGrabberBack.setPosition(0);
        } else if (skystoneGrabberMode == "|-- \n|") {
            drive.skystoneArmFront.setPosition(0);
            drive.skystoneGrabberFront.setPosition(1);
            drive.skystoneArmBack.setPosition(0);
            drive.skystoneGrabberBack.setPosition(1);
        } else if (skystoneGrabberMode == "\n----|") {
            drive.skystoneArmFront.setPosition(skystoneArmDownPosition);
            drive.skystoneGrabberFront.setPosition(1);
            drive.skystoneArmBack.setPosition(skystoneArmDownPosition);
            drive.skystoneGrabberBack.setPosition(1);
        } else if (skystoneGrabberMode == "\n--[]") {
            drive.skystoneArmFront.setPosition(skystoneArmDownPosition);
            drive.skystoneGrabberFront.setPosition(0);
            drive.skystoneArmBack.setPosition(skystoneArmDownPosition);
            drive.skystoneGrabberBack.setPosition(0);
        } else if (skystoneGrabberMode == "|[] \n|") {
            drive.skystoneArmFront.setPosition(0);
            drive.skystoneGrabberFront.setPosition(0);
            drive.skystoneArmBack.setPosition(0);
            drive.skystoneGrabberBack.setPosition(0);
        } else if (skystoneGrabberMode == "\n--[_]") {
            drive.skystoneArmFront.setPosition(skystoneArmDownPosition - 0.05);
            drive.skystoneGrabberFront.setPosition(0);
            drive.skystoneArmBack.setPosition(skystoneArmDownPosition - 0.05);
            drive.skystoneGrabberBack.setPosition(0);
        } else if (skystoneGrabberMode == "\n--_--") {
            drive.skystoneArmFront.setPosition(skystoneArmDownPosition - 0.05);
            drive.skystoneGrabberFront.setPosition(1);
            drive.skystoneArmBack.setPosition(skystoneArmDownPosition - 0.05);
            drive.skystoneGrabberBack.setPosition(1);
        }

    }

}

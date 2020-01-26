package org.firstinspires.ftc.teamcode.yoda_opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yoda_code.YodaMecanumDrive;

import java.util.List;

@TeleOp(group = "drive")
public class ManualDrive extends LinearOpMode {
    private YodaMecanumDrive drive;
    private boolean[] pressed = new boolean[10];

    private double speed_multiplier = 1;
    private boolean isSlowMode = false;
    private double FAST_MODE_MULTIPLIER = 1.4; //change to 1.0 for now but it only get 0.7 power. Try using 1.4, but it disables other directions
    private double SLOW_MODE_MULTIPLIER = 0.5;
    private double TURNING_SPEED = 1;
    private double DPAD_SPEED = 0.2;

    private String stoneGrabberMode = "|| \n|";
    private String capstoneArmMode = "Stored";
    private double capstonePosition = 0;

    private double intakeGrabberPosition = 0;
    private double leftFoundationMoverPosition = 0;
    private double rightFoundationMoverPosition = 0;

    private double verticalPosition = 0;
    private double previousVerticalPos = -1;
    private final double bottomVerticalLim = 0 - 10;
    private final double topVerticalLim = 2800;
    private double lastTopPosition = 20;
    private final double ticksPerUpBlock = 550;
    private final double ticksPerDownBlock = 200;

    private double horizontalPosition = 0;
    private double previousHorizontalPos = -1;
    private final double placingHorizontalPos = 0.75;
    private final double capstonePositionThreshold = 0.25;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new YodaMecanumDrive(hardwareMap);
        drive.setOpMode(this);
        drive.resetTimer();
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        horizontalPosition = 0.99;
        previousHorizontalPos = 0.99;

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clear();
        waitForStart();

        if (isStopRequested()) return;
        drive.setMotorNoEncoder();
//        ElapsedTime op_timer = new ElapsedTime();

        drive.foundationMoverLeft.setPosition(0);
        drive.foundationMoverRight.setPosition(0);
        drive.intakeGrabber.setPosition(0); // open, set power so that it does not go down

        while (!isStopRequested()) {
//            op_timer.reset();

            moveRobot();
            controlIntakeGrabber();
            controlCapstone();
            controlVerticalExtender();
            controlHorizontalExtender();
            moveFoundationServo();
            controlSkystoneGrabbers();
            controlParkingArm();

//            telemetry.addData("Encoders", "Left %d, Right %d, Front %d", drive.leftEncoder.getCurrentPosition(), drive.rightEncoder.getCurrentPosition(), drive.frontEncoder.getCurrentPosition());
//            telemetry.addData("Speed co-efficients", "turn %.2f   ||   entire %.2f", TURNING_SPEED, speed_multiplier);
//            telemetry.addData("Encoder values, lf, lr, rr, rf", drive.getWheelPositions());
//            telemetry.addData("Capstone Arm", capstoneArmMode);
            telemetry.addData("Capstone Pos", capstonePosition);

            telemetry.addData("Horizontal Pos", horizontalPosition);
//            telemetry.addData("Vertical Pos", "I: " + verticalPosition + " - O: " + drive.verticalExtender.getCurrentPosition());
            telemetry.addData("Intake Pos", intakeGrabberPosition);
            telemetry.addData("Stone Grabber", "\n" + stoneGrabberMode + "\n|");
//            telemetry.addData("Latency", op_timer.milliseconds());
            telemetry.update();
        }
    }

    private void moveRobot() {
        if (gamepad1.a && !pressed[1]) {
            isSlowMode = !isSlowMode;
            pressed[1] = true;
        } else if (!gamepad1.a && pressed[1]) {
            pressed[1] = false;
        }

        // does not work, disable now

        //if (gamepad1.x && !pressed[0]) {
        //    drive.turnSync(drive.getAngleToFront(telemetry));
        //    pressed[0] = true;
        //} else if (!gamepad1.x && pressed[0]) {
        //    pressed[0] = false;
        //}

        //if (gamepad1.x) drive.turnSync(drive.getAngleToFront(telemetry));

        double leftFrontPower, rightFrontPower, leftRearPower, rightRearPower;

        double input_x = drive.pow(gamepad1.left_stick_x, 1.1);
        double input_y = drive.pow(-gamepad1.left_stick_y, 1.1);
        double input_turning = drive.pow(gamepad1.right_stick_x, 1.1);

//        telemetry.addData("Initial Input:", "x %.2f, y %.2f, turning %.2f", input_x, input_y, input_turning);

        if (isSlowMode) {
            speed_multiplier = SLOW_MODE_MULTIPLIER;
            drive.setLed(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        else {
            speed_multiplier = FAST_MODE_MULTIPLIER;
            drive.setLed(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }

        if (gamepad1.dpad_up) {
            input_y += DPAD_SPEED / speed_multiplier;
        } if (gamepad1.dpad_down) {
            input_y -= DPAD_SPEED / speed_multiplier;
        }

        if (gamepad1.dpad_right) {
            input_x += 2 * DPAD_SPEED / speed_multiplier;
        } if (gamepad1.dpad_left) {
            input_x -= 2 * DPAD_SPEED / speed_multiplier;
        }

        if (gamepad1.left_bumper) {
            input_turning -= DPAD_SPEED * 0.65 / speed_multiplier;
        } if (gamepad1.right_bumper) {
            input_turning += DPAD_SPEED * 0.65 / speed_multiplier;
        }

//        telemetry.addData("Input:", "x %.2f, y %.2f, turning %.2f", input_x, input_y, input_turning);

        double r = Math.hypot(input_x, input_y);
        double robotAngle = Math.atan2(input_y, input_x) - Math.PI / 4; // Math.PI/4 is the equivalent of 45 degrees

        leftFrontPower = (r * Math.cos(robotAngle) + TURNING_SPEED * input_turning) * speed_multiplier;
        rightFrontPower = (r * Math.sin(robotAngle) - TURNING_SPEED * input_turning) * speed_multiplier;
        leftRearPower = (r * Math.sin(robotAngle) + TURNING_SPEED * input_turning) * speed_multiplier;
        rightRearPower = (r * Math.cos(robotAngle) - TURNING_SPEED * input_turning) * speed_multiplier;

        List<Double> powersList = drive.scaleDown(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower, 1);

        drive.setMotorPowers(powersList.get(0), powersList.get(1), powersList.get(2), powersList.get(3));
    }

    private void moveFoundationServo() {
        if (gamepad1.left_trigger > 0.2 && !pressed[8]) {
            leftFoundationMoverPosition = (leftFoundationMoverPosition == 0) ? 1 : 0;
            pressed[8] = true;
        }
        if (gamepad1.left_trigger <= 0.2 && pressed[8]) pressed[8] = false;

        if (gamepad1.right_trigger > 0.2 && !pressed[9]) {
            rightFoundationMoverPosition = (rightFoundationMoverPosition == 0) ? 1 : 0;
            pressed[9] = true;
        }
        if (gamepad1.right_trigger <= 0.2 && pressed[9]) pressed[9] = false;

        if (gamepad1.x) {
            leftFoundationMoverPosition = 0;
            rightFoundationMoverPosition = 0;
        }

        drive.foundationMoverLeft.setPosition(leftFoundationMoverPosition);
        drive.foundationMoverRight.setPosition(rightFoundationMoverPosition);
    }

    private void controlIntakeGrabber() {
        if (gamepad2.a) { // grab
            intakeGrabberPosition = 1;
        }
        if (gamepad2.b) { // release
            if (verticalPosition >= 20) intakeGrabberPosition= 0.2;
            else intakeGrabberPosition = 0.3;

            if (verticalPosition >= 20) {
                lastTopPosition = verticalPosition;
            }
        }
//        if (gamepad2.left_trigger > 0.1) intakeGrabberPosition = Math.min(intakeGrabberPosition + 0.005, 1);
//        if (gamepad2.right_trigger > 0.1) intakeGrabberPosition = Math.max(intakeGrabberPosition - 0.005, 0);
        drive.intakeGrabber.setPosition(intakeGrabberPosition);
    }

    private void controlCapstone() {
        if (gamepad2.x && !pressed[6]) {
            if (capstoneArmMode == "Stored") capstoneArmMode = "Ready";
            else if (capstoneArmMode == "Ready") capstoneArmMode = "Stored";

            if (capstoneArmMode == "Stored") {
                capstonePosition = 0;
            } else if (capstoneArmMode == "Ready") {
                capstonePosition = 0.7;
            }

            pressed[6] = true;
        } else if (!gamepad2.x && pressed[6]) {
            pressed[6] = false;
        }

        if (gamepad2.left_bumper) capstonePosition = Math.min(capstonePosition + 0.0015, 1);
        if (gamepad2.right_bumper) capstonePosition = Math.max(capstonePosition - 0.0015, 0);

        if (capstonePosition >= capstonePositionThreshold) {
            intakeGrabberPosition = 0;
            horizontalPosition = 0;
        }
        drive.capstoneArm.setPosition(capstonePosition);
    }

    private void controlParkingArm() {
        if (gamepad2.y && !pressed[3]) {
            if (drive.parkingArm.getPosition() == 1) {
                drive.parkingArm.setPosition(0);
            }
            else {
                drive.parkingArm.setPosition(1);
            }
            pressed[3] = true;
        } else if (!gamepad2.y && pressed[3]) {
            pressed[3] = false;
        }

        drive.parkingTape.setPower(Math.pow(gamepad2.right_trigger, 3) - Math.pow(gamepad2.left_trigger, 3));

    }

    private void controlVerticalExtender() {
        if ((gamepad2.dpad_up && !gamepad2.dpad_left && !gamepad2.dpad_right) && !pressed[4] && capstonePosition < capstonePositionThreshold) {
            verticalPosition = lastTopPosition + ticksPerUpBlock;
            pressed[4] = true;
        } else if (!gamepad2.dpad_up && pressed[4]) {
            pressed[4] = false;
        }

//        if (gamepad2.dpad_down && !pressed[5] && capstonePosition < capstonePositionThreshold) {
//            verticalPosition -= ticksPerDownBlock;
//            pressed[5] = true;
//        } else if (!gamepad2.dpad_down && pressed[5]) {
//            pressed[5] = false;
//        }

        if (gamepad2.dpad_left) verticalPosition -= 3;
        if (gamepad2.dpad_right) verticalPosition += 3;

        if (gamepad2.right_stick_button) verticalPosition = bottomVerticalLim;

        verticalPosition = verticalPosition - gamepad2.right_stick_y * 15 * ((capstonePosition >= capstonePositionThreshold) ? 0.25 : 1.0);

        if (!(gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_down)) {
            verticalPosition = drive.minMax(verticalPosition, bottomVerticalLim, topVerticalLim);
        }
        if (verticalPosition != previousVerticalPos) drive.verticalExtender.setTargetPosition((int) verticalPosition);
        previousVerticalPos = verticalPosition;
    }

    private void controlHorizontalExtender() {

        if (gamepad2.left_stick_button) horizontalPosition = placingHorizontalPos;
        double input_y = gamepad2.left_stick_y;
        if (Math.abs(input_y) < 0.1) {
            // Skip too small input from joystick as there might be noise
            input_y = 0;
        }

        horizontalPosition = drive.minMax(horizontalPosition - input_y / 25, 0, 1);
        if (horizontalPosition != previousHorizontalPos) {
            //drive.log("Horizontal Position set to: " + horizontalPosition);
            drive.horizontalExtender.setPosition(horizontalPosition);
        }
        previousHorizontalPos = horizontalPosition;
    }

    private void controlSkystoneGrabbers() {
        if (gamepad1.y && !pressed[2]) {
            if (stoneGrabberMode == "|| \n|") stoneGrabberMode = "|-- \n|";
            else if (stoneGrabberMode == "|-- \n|") stoneGrabberMode = "\n----|";
            else if (stoneGrabberMode == "\n----|") stoneGrabberMode = "\n--[]";
            else if (stoneGrabberMode == "\n--[]") stoneGrabberMode = "|[] \n|";
            else if (stoneGrabberMode == "|[] \n|") stoneGrabberMode = "\n--[_]";
            else if (stoneGrabberMode == "\n--[_]") stoneGrabberMode = "\n--_--";
            else if (stoneGrabberMode == "\n--_--") stoneGrabberMode = "|| \n|";
            pressed[2] = true;
        } else if (!gamepad1.y && pressed[2]) {
            pressed[2] = false;
        }

        if (stoneGrabberMode == "|| \n|") { // Stored
            drive.skystoneArmFront.setPosition(0);
            drive.skystoneGrabberFront.setPosition(0);
            drive.skystoneArmBack.setPosition(0);
            drive.skystoneGrabberBack.setPosition(0);
        } else if (stoneGrabberMode == "|-- \n|") { // Open grabbers
            drive.skystoneGrabberFront.setPosition(1);
            drive.skystoneGrabberBack.setPosition(1);
        } else if (stoneGrabberMode == "\n----|") { // Move arm down
            drive.skystoneArmFront.setPosition(1);
            drive.skystoneArmBack.setPosition(1);
        } else if (stoneGrabberMode == "\n--[]") { // Grab
            drive.skystoneGrabberFront.setPosition(0);
            drive.skystoneGrabberBack.setPosition(0);
        } else if (stoneGrabberMode == "|[] \n|") { // Move arm back up
            drive.skystoneArmFront.setPosition(0);
            drive.skystoneArmBack.setPosition(0);
        } else if (stoneGrabberMode == "\n--[_]") { // Move arm back down
            drive.skystoneArmFront.setPosition(1 - 0.14);
            drive.skystoneArmBack.setPosition(1 - 0.14);
        } else if (stoneGrabberMode == "\n--_--") { // Release grabbers
            drive.skystoneGrabberFront.setPosition(1);
            drive.skystoneGrabberBack.setPosition(1);
        }

    }
}

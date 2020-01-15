package org.firstinspires.ftc.teamcode.yoda_opmode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yoda_code.YodaMecanumDrive;

import java.util.List;

@TeleOp(group = "drive")
public class ManualDrive extends LinearOpMode {
    private YodaMecanumDrive drive;
    private boolean[] pressed = new boolean[8];

    private double speed_multiplier = 1;
    private boolean isSlowMode = false;
    private boolean isStopMode = false;
    private double FAST_MODE_MULTIPLIER = 1.4; //change to 1.0 for now but it only get 0.7 power. Try using 1.4, but it disables other directions
    private double SLOW_MODE_MULTIPLIER = 0.5;
    private double TURNING_SPEED = 1;
    private double DPAD_SPEED = 0.2;

    private String stoneGrabberMode = "|| \n|";
    private String capstoneArmMode = "Stored";
    private double capstonePosition = 0;

    private double intakeGrabberPosition = 0;
    private double foundationMoverPosition = 0;

    private double verticalPosition = 0;
    private double previousVerticalPos = -1;
    private final double bottomVerticalLim = -10;
    private final double topVerticalLim = 2542;
    private double lastTopPosition = 20;
    private final double ticksPerUpBlock = 550;
    private final double ticksPerDownBlock = 200;

    private double horizontalPosition = 0;
    private double previousHorizontalPos = -1;
    private final double placingHorizontalPos = 0.75;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new YodaMecanumDrive(hardwareMap);
        drive.setOpMode(this);
        drive.resetTimer();
        // telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        horizontalPosition = 0.99;
        previousHorizontalPos = 0.99;
        gamepad1.setJoystickDeadzone(0.01f);
        gamepad2.setJoystickDeadzone(0.01f);

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clear();
        waitForStart();

        if (isStopRequested()) return;
        drive.setMotorNoEncoder();
        ElapsedTime op_timer = new ElapsedTime();

        drive.foundationMoverLeft.setPosition(0);
        drive.foundationMoverRight.setPosition(0);
        drive.intakeGrabber.setPosition(0); // open, set power so that it does not go down

        while (!isStopRequested()) {
            op_timer.reset();

            moveRobot();
            controlIntakeGrabber();
            controlCapstone();
            controlVerticalExtender();
            controlHorizontalExtender();
            moveFoundationServo();
            controlSkystoneGrabbers();
            controlParkingArm();

//            telemetry.addData("Encoders", "Left %d, Right %d, Front %d", drive.leftEncoder.getCurrentPosition(), drive.rightEncoder.getCurrentPosition(), drive.frontEncoder.getCurrentPosition());
            telemetry.addData("Speed co-efficients", "turn %.2f   ||   entire %.2f", TURNING_SPEED, speed_multiplier);
//            telemetry.addData("Encoder values, lf, lr, rr, rf", drive.getWheelPositions());
            telemetry.addData("Capstone Arm", capstoneArmMode);
            telemetry.addData("Capstone Pos", capstonePosition);

            telemetry.addData("Horizontal Pos", horizontalPosition);
            telemetry.addData("Vertical Pos", verticalPosition + " - " + drive.verticalExtender.getCurrentPosition());
            telemetry.addData("Intake Pos", intakeGrabberPosition);
//            telemetry.addData("Stone Grabber", "\n" + stoneGrabberMode + "\n|");
            telemetry.addData("Latency", op_timer.milliseconds());
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

        if (gamepad1.x && !pressed[7]) {
            isStopMode = !isStopMode;
            isSlowMode = true;
            pressed[7] = true;
        } else if (!gamepad1.x && pressed[7]) {
            pressed[7] = false;
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

        if (isStopMode) {
            speed_multiplier = SLOW_MODE_MULTIPLIER - 0.1;
            drive.setLed(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
        }
        else if (isSlowMode) {
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

        telemetry.addData("Input:", "x %.2f, y %.2f, turning %.2f", input_x, input_y, input_turning);

        if (isStopMode && input_y > 0 && foundationMoverPosition == 0) {
            double frontShort = Math.min(drive.getFrontLeftDistance(), drive.getFrontRightDistance());
            frontShort = Math.max(frontShort, 0);
            if (frontShort <= 10) {
                input_y = drive.pow(frontShort / 10, 0.8) * input_y;
            }
        }

        double r = Math.hypot(input_x, input_y);
        double robotAngle = Math.atan2(input_y, input_x) - Math.PI / 4; // Math.PI/4 is the equivalent of 45 degrees

        leftFrontPower = (r * Math.cos(robotAngle) + TURNING_SPEED * input_turning) * speed_multiplier;
        rightFrontPower = (r * Math.sin(robotAngle) - TURNING_SPEED * input_turning) * speed_multiplier;
        leftRearPower = (r * Math.sin(robotAngle) + TURNING_SPEED * input_turning) * speed_multiplier;
        rightRearPower = (r * Math.cos(robotAngle) - TURNING_SPEED * input_turning) * speed_multiplier;

        List<Double> powersList = drive.scaleDown(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower, 1);

        telemetry.addData("Powers", powersList);
        drive.setMotorPowers(powersList.get(0), powersList.get(1), powersList.get(2), powersList.get(3));

//        telemetry.addData("Wheel Powers", "lf %.2f, lr %.2f, rr %.2f, rf %.2f", powersList.get(0), powersList.get(1), powersList.get(2), powersList.get(3));
//        drive.log("Left Front " + leftFrontPower + " > " + powersList.get(0));
//        drive.log("Left Rear " + leftRearPower + " > " + powersList.get(1));
//        drive.log("Right Rear " + rightRearPower + " > " + powersList.get(2));
//        drive.log("Right Front " + rightFrontPower + " > " + powersList.get(3));
//        drive.log("");
    }

    private void moveFoundationServo() {
        if (gamepad1.left_trigger >= 0.1) foundationMoverPosition = 1;
        else if (gamepad1.right_trigger >= 0.5) foundationMoverPosition = 0;

        drive.foundationMoverLeft.setPosition(foundationMoverPosition);
        drive.foundationMoverRight.setPosition(foundationMoverPosition);
    }

    private void controlIntakeGrabber() {
        if (gamepad2.a) { // grab
            intakeGrabberPosition = 1;
        }
        if (gamepad2.b) { // release
            intakeGrabberPosition = 0.35;

            if (verticalPosition >= 20) {
                lastTopPosition = verticalPosition;
            }
        }
        if (gamepad2.left_trigger > 0.1) intakeGrabberPosition = Math.min(intakeGrabberPosition + 0.005, 1);
        if (gamepad2.right_trigger > 0.1) intakeGrabberPosition = Math.max(intakeGrabberPosition - 0.005, 0);
        drive.intakeGrabber.setPosition(intakeGrabberPosition);
    }

    private void controlCapstone() {
        if (gamepad2.x && !pressed[6]) {
            if (capstoneArmMode == "Stored") capstoneArmMode = "Ready";
            else if (capstoneArmMode == "Ready") capstoneArmMode = "Dropped";
            else if (capstoneArmMode == "Dropped") capstoneArmMode = "Stored";

            if (capstoneArmMode == "Stored") {
                capstonePosition = 0;
            } else if (capstoneArmMode == "Ready") {
                capstonePosition = 0.55;
                intakeGrabberPosition = 0;
                horizontalPosition = 0;
            } else if (capstoneArmMode == "Dropped") {
               capstonePosition = 0.7;
            }

            pressed[6] = true;
        } else if (!gamepad2.x && pressed[6]) {
            pressed[6] = false;
        }

        if (gamepad2.left_bumper) capstonePosition = Math.min(capstonePosition + 0.005, 1);
        if (gamepad2.right_bumper) capstonePosition = Math.max(capstonePosition - 0.005, 0);

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
    }

    private void controlVerticalExtender() {
        if (gamepad2.dpad_up && !pressed[4]) {
            verticalPosition = lastTopPosition + ticksPerUpBlock;
            pressed[4] = true;
        } else if (!gamepad2.dpad_up && pressed[4]) {
            pressed[4] = false;
        }

        if (gamepad2.dpad_down && !pressed[5]) {
            verticalPosition -= ticksPerDownBlock;
            pressed[5] = true;
        } else if (!gamepad2.dpad_down && pressed[5]) {
            pressed[5] = false;
        }

        if (gamepad2.dpad_left) verticalPosition -= 2;
        if (gamepad2.dpad_right) verticalPosition += 2;

        if (gamepad2.right_stick_button) verticalPosition = bottomVerticalLim;

        verticalPosition = Math.min(Math.max(verticalPosition - gamepad2.right_stick_y * 25, bottomVerticalLim), topVerticalLim);

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

        horizontalPosition = drive.keepBetweenMaxMin(horizontalPosition - input_y / 25, 1, 0);
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

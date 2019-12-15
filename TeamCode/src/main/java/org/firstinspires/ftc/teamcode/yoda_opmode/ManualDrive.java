package org.firstinspires.ftc.teamcode.yoda_opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.yoda_code.YodaMecanumDrive;

import java.util.List;

@TeleOp(group = "drive")
public class ManualDrive extends LinearOpMode {
    private YodaMecanumDrive drive;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private boolean[] pressed = new boolean[6];

    private double speed_multiplier = 1;
    private boolean isSlowMode = false;
    private double GLOBAL_SPEED_MULTIPLIER = 1.4; //change to 1.0 for now but it only get 0.7 power. Try using 1.4, but it disables other directions
    private double SLOW_MODE_MULTIPLIER = 0.5;
    private double TURNING_SPEED = 1;
    private double DPAD_SPEED = 0.2;

    private String stoneGrabberMode = "|| \n|";
    private String capstoneArmMode = "Stored";

    private double verticalPosition = 0;
    private double previousVerticalPos = -1;
    private final double bottomVerticalLim = -20;
    private final double topVerticalLim = 2542;
    private double lastTopPosition = 20;
    private final double ticksPerUpBlock = 300;
    private final double ticksPerDownBlock = 230;

    private double horizontalPosition = 0;
    private double previousHorizontalPos = -1;
    private final double placingHorizontalPos = 0.9;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new YodaMecanumDrive(hardwareMap);
        drive.setOpMode(this);
        drive.resetTimer();
        // telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        horizontalPosition = 1;
        previousHorizontalPos = 1;

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clear();
        waitForStart();

        if (isStopRequested()) return;
        ElapsedTime op_timer = new ElapsedTime();

        while (!isStopRequested()) {
            op_timer.reset();

            moveRobot();
            controlIntakeGrabber();
            controlCapstone();
            controlVerticalExtender();
            controlHorizontalExtender();
            moveFoundationServo();
            controlSkystoneGrabbers();

            telemetry.addData("Encoders", "Left %d, Right %d, Front %d", drive.leftEncoder.getCurrentPosition(), drive.rightEncoder.getCurrentPosition(), drive.frontEncoder.getCurrentPosition());
            telemetry.addData("Speed co-efficients", "turn %.2f   ||   entire %.2f", TURNING_SPEED, speed_multiplier);
            telemetry.addData("Encoder values, lf, lr, rr, rf", drive.getWheelPositions());
            //disable because they are slow
//            telemetry.addData("Distance", "Left %.2f, Right %.2f", drive.frontLeftDistance.getDistance(DistanceUnit.INCH), drive.frontRightDistance.getDistance(DistanceUnit.INCH));
//            telemetry.addData("Front Angle", Math.toDegrees(drive.getAngleToFront()));
            telemetry.addData("Capstone Arm", capstoneArmMode);
//            telemetry.addData("Stone Grabber", "\n" + stoneGrabberMode + "\n|");
            telemetry.addData("Horizontal Pos", drive.horizontalExtender.getPosition());
            telemetry.addData("Vertical Pos", drive.verticalExtender.getCurrentPosition());
            telemetry.addData("Latency", op_timer.milliseconds());
            telemetry.update();
        }
    }

    private void moveFoundationServo() {
        if (gamepad1.left_trigger >= 0.5 || gamepad1.right_trigger >= 0.5) {
            drive.foundationMoverLeft.setPosition(1);
            drive.foundationMoverRight.setPosition(1);
            drive.intakeGrabber.setPosition(0.52);
        }
        else {
            drive.foundationMoverLeft.setPosition(0);
            drive.foundationMoverRight.setPosition(0);
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

        double leftFrontPower = 0;
        double rightFrontPower = 0;
        double leftRearPower = 0;
        double rightRearPower = 0;

        double input_x = gamepad1.left_stick_x;
        double input_y = -gamepad1.left_stick_y;
        double input_turning = gamepad1.right_stick_x;

//        telemetry.addData("Initial Input:", "x %.2f, y %.2f, turning %.2f", input_x, input_y, input_turning);

        // Skip too small input from joystick as they might be noise
        // SetJoystickDeadzone does not work, has to do ourselves.
        if (Math.abs(input_x) < 0.15) {
            input_x = 0;
        }

        if (Math.abs(input_y) < 0.15) {
            input_y = 0;
        }

        if (Math.abs(input_turning) < 0.15) {
            input_turning = 0;
        }

        boolean dpad_pressed = false;
        if (gamepad1.dpad_up) {
            input_y += DPAD_SPEED;
            dpad_pressed = true;
        } else if (gamepad1.dpad_down) {
            input_y -= DPAD_SPEED;
            dpad_pressed = true;
        }

        if (gamepad1.dpad_right) {
            input_x += DPAD_SPEED;
            dpad_pressed = true;
        } else if (gamepad1.dpad_left) {
            input_x -= DPAD_SPEED;
            dpad_pressed = true;
        }

        if (gamepad1.left_bumper) {
            input_turning -= DPAD_SPEED * 0.65;
            dpad_pressed = true;
        } else if (gamepad1.right_bumper) {
            input_turning += DPAD_SPEED * 0.65;
            dpad_pressed = true;
        }

        telemetry.addData("Input:", "x %.2f, y %.2f, turning %.2f", input_x, input_y, input_turning);

        if ((isSlowMode && !dpad_pressed)|| gamepad2.y) {
            speed_multiplier = SLOW_MODE_MULTIPLIER;
        } else {
            speed_multiplier = 1 * GLOBAL_SPEED_MULTIPLIER;
        }

        if ((gamepad2.left_trigger > 0.9 || gamepad2.right_trigger > 0.9) && input_y > 0) {
            if (drive.frontLeftDistance.getDistance(DistanceUnit.INCH) < 10 || drive.frontRightDistance.getDistance(DistanceUnit.INCH) < 10) {
                input_y = 0;
            }
        }


        double r = Math.hypot(input_x, input_y);
        double robotAngle = Math.atan2(input_y, input_x) - Math.PI / 4;

        leftFrontPower = (r * Math.cos(robotAngle) + TURNING_SPEED * input_turning) * speed_multiplier;
        rightFrontPower = (r * Math.sin(robotAngle) - TURNING_SPEED * input_turning) * speed_multiplier;
        leftRearPower = (r * Math.sin(robotAngle) + TURNING_SPEED * input_turning) * speed_multiplier;
        rightRearPower = (r * Math.cos(robotAngle) - TURNING_SPEED * input_turning) * speed_multiplier;

        List<Double> powersList = drive.scaleDown(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower, 1);

        drive.setMotorPowers(powersList.get(0), powersList.get(1), powersList.get(2), powersList.get(3));

//        telemetry.addData("Wheel Powers", "lf %.2f, lr %.2f, rr %.2f, rf %.2f", powersList.get(0), powersList.get(1), powersList.get(2), powersList.get(3));
//        drive.log("Left Front " + leftFrontPower + " > " + powersList.get(0));
//        drive.log("Left Rear " + leftRearPower + " > " + powersList.get(1));
//        drive.log("Right Rear " + rightRearPower + " > " + powersList.get(2));
//        drive.log("Right Front " + rightFrontPower + " > " + powersList.get(3));
//        drive.log("");
    }

    private void controlIntakeGrabber() {
        if (gamepad2.left_bumper || gamepad2.a) drive.intakeGrabber.setPosition(0); // close
        if (gamepad2.right_bumper || gamepad2.b) {
            drive.intakeGrabber.setPosition(0.4); // open

            if (verticalPosition >= 20) {
                lastTopPosition = verticalPosition;
            }
        }
    }

    private void controlCapstone() {
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
            drive.capstoneArm.setPosition(0.25);
        } else if (capstoneArmMode == "Ready") {
            drive.capstoneArm.setPosition(0.92);
            drive.intakeGrabber.setPosition(0.45);
            drive.horizontalExtender.setPosition(0);
            horizontalPosition = 0;
        } else if (capstoneArmMode == "Dropping") {
            drive.capstoneArm.setPosition(drive.capstoneArm.getPosition() + 0.002);
        } else if (capstoneArmMode == "Dropped") {
            drive.capstoneArm.setPosition(0.99);
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
            drive.skystoneArmFront.setPosition(1 - 0.15);
            drive.skystoneArmBack.setPosition(1 - 0.15);
        } else if (stoneGrabberMode == "\n--_--") { // Release grabbers
            drive.skystoneGrabberFront.setPosition(1);
            drive.skystoneGrabberBack.setPosition(1);
        }

    }
}

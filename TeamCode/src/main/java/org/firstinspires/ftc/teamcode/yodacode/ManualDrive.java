package org.firstinspires.ftc.teamcode.yodacode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "drive")
public class ManualDrive extends LinearOpMode {
    private YodaMecanumDrive drive;
    private boolean[] pressed = new boolean[4];
    private double speed = 1;
    private double turningSpeed = 0.9;
    private double dPadAdditionX = 0;
    private double dPadAdditionY = 0;
    private double bumperTurnAddition = 0;
    private String foundationMoverMode = "Stored";
    private String skystoneGrabberMode = "|| \n|";
    private String capstoneArmMode = "Stored";
    private double horizontalPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new YodaMecanumDrive(hardwareMap);
        drive.setTelemetry(telemetry);


        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clear();
        gamepad1.setJoystickDeadzone(0.05f);
        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {

            moveRobot();
            intake();
            controlExtenders();
            moveFoundationServo();

            telemetry.addData("Speed co-efficients", "turn *%.2f, entire *%.2f", turningSpeed, speed);
            telemetry.addData("Encoder values, lf, lr, rr, rf", drive.getWheelPositions());
//            telemetry.addData(
//                    "Distance",
//                    "front: %.2f, back: %.2f",
//                    drive.frontDistance.getDistance(DistanceUnit.INCH),
//                    drive.backDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Foundation Mover", foundationMoverMode);
            telemetry.addData("Capstone Arm", capstoneArmMode);
            telemetry.addData("Skystone Grabber", "\n" + skystoneGrabberMode + "|");
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
            drive.foundationMoverLeft.setPosition(0.5);
            drive.foundationMoverRight.setPosition(0);
        } else if (foundationMoverMode == "Grab") {
            drive.foundationMoverLeft.setPosition(0.8);
            drive.foundationMoverRight.setPosition(0.8);
        }
    }

    private void moveRobot() {
        if (gamepad1.a && !pressed[1]) {
            if (speed == 0.5) speed = 1;
            else if (speed == 1) speed = 0.5;
            pressed[1] = true;
        }
        else if (!gamepad1.a && pressed[1]) {
            pressed[1] = false;
        }

        if (gamepad1.dpad_right || gamepad2.dpad_right) dPadAdditionX = -0.2 * 2;
        else if (gamepad1.dpad_left || gamepad2.dpad_left) dPadAdditionX = 0.2 * 2;

        if (gamepad1.dpad_up || gamepad2.dpad_up) dPadAdditionY = 0.2;
        else if (gamepad1.dpad_down || gamepad2.dpad_down) dPadAdditionY = -0.2;

        if (!gamepad1.left_bumper && !gamepad1.right_bumper) bumperTurnAddition = 0;
        else if (gamepad1.left_bumper) bumperTurnAddition = 0.15;
        else if (gamepad1.right_bumper) bumperTurnAddition = -0.15;


        if (!(gamepad1.dpad_right || gamepad2.dpad_right || gamepad1.dpad_left || gamepad2.dpad_left)) {
            dPadAdditionX = 0;
        }
        if (!(gamepad1.dpad_up || gamepad2.dpad_up || gamepad1.dpad_down || gamepad2.dpad_down)) {
            dPadAdditionY = 0;
        }

        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;

        double leftFrontPower = (r * Math.cos(robotAngle) + turningSpeed * rightX) * speed + dPadAdditionY - dPadAdditionX - bumperTurnAddition;
        double rightFrontPower = (r * Math.sin(robotAngle) - turningSpeed * rightX) * speed + dPadAdditionY + dPadAdditionX + bumperTurnAddition;
        double leftRearPower = (r * Math.sin(robotAngle) + turningSpeed * rightX) * speed + dPadAdditionY + dPadAdditionX - bumperTurnAddition;
        double rightRearPower = (r * Math.cos(robotAngle) - turningSpeed * rightX) * speed + dPadAdditionY - dPadAdditionX + bumperTurnAddition;

        drive.setMotorPowers(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);
    }

    private void intake() {
        if (gamepad2.a) drive.intakeGrabber.setPosition(1);
        if (gamepad2.b) drive.intakeGrabber.setPosition(0);

        if (gamepad2.x && !pressed[3]) {
            if      (capstoneArmMode == "Stored") capstoneArmMode = "Ready";
            else if (capstoneArmMode == "Ready") capstoneArmMode = "Dropping";
            else if (capstoneArmMode == "Dropping") capstoneArmMode = "Dropped";
            else if (capstoneArmMode == "Dropped") capstoneArmMode = "Stored";
            pressed[3] = true;
        }
        else if (!gamepad2.x && pressed[3]) {
            pressed[3] = false;
        }
        if (capstoneArmMode == "Dropping" && drive.capstoneArm.getPosition() < 0.9) capstoneArmMode = "Dropped";

        if (capstoneArmMode == "Stored") {
            drive.capstoneArm.setPosition(0);
        }
        else if (capstoneArmMode == "Ready") {
            drive.capstoneArm.setPosition(0.5);
        }
        else if (capstoneArmMode == "Dropping") {
            drive.capstoneArm.setPosition(drive.capstoneArm.getPosition() + 0.005);
        }
        else if (capstoneArmMode == "Dropped") {
            drive.capstoneArm.setPosition(0.9);
        }

    }

    private void controlExtenders() {
        double verticalPower = -gamepad2.right_stick_y;
        if (verticalPower == 0 && drive.verticalExtender.getCurrentPosition() > 20) {verticalPower = 0.05;}
        drive.verticalExtender.setPower(verticalPower);
        drive.horizontalExtender.setPosition(horizontalPosition);

        horizontalPosition = Math.min(Math.max( horizontalPosition - gamepad2.left_stick_y / 100 , 1), 1);
    }

    private void controlSkystoneGrabbers() {
        if (gamepad1.y && !pressed[2]) {
                 if (skystoneGrabberMode == "|| \n|") skystoneGrabberMode = "|-- \n|";
            else if (skystoneGrabberMode == "|-- \n|") skystoneGrabberMode = "\n----|";
            else if (skystoneGrabberMode == "\n----|") skystoneGrabberMode = "\n--__";
            else if (skystoneGrabberMode == "\n--__") skystoneGrabberMode = "|| \n|";
            pressed[2] = true;
        }
        else if (!gamepad1.y && pressed[2]) {
            pressed[2] = false;
        }

        if (skystoneGrabberMode == "|| \n|") {

        }
        else if (skystoneGrabberMode == "|-- \n|") {

        }
        else if (skystoneGrabberMode == "\n----|") {

        }
        else if (skystoneGrabberMode == "\n--__") {

        }

    }

}

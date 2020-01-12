package org.firstinspires.ftc.teamcode.yoda_opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.yoda_code.AutonomousBase;
import org.firstinspires.ftc.teamcode.yoda_code.YodaMecanumDrive;
import org.firstinspires.ftc.teamcode.yoda_enum.TeamColor;

//@Disabled
@Config
@TeleOp(group = "Test", name = "Servo Tester")
public class ServoTesting extends LinearOpMode {
    private YodaMecanumDrive drive;

    public static double servoPosition = 0;
    private boolean pressed = false;
    private String[] servos= {"horizontal extender", "left foundation mover", "right foundation mover", "front skystone arm", "back skystone arm", "front skystone grabber", "back skystone grabber", "capstone arm", "parking arm"};
    private int servoNum = 0;
    private Servo previousServo;
    private Servo targetServo;
    public static boolean update = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new YodaMecanumDrive(hardwareMap);
        drive.setOpMode(this);
        drive.resetTimer();

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clear();
        waitForStart();

        drive.setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (isStopRequested()) return;
        while (!isStopRequested()) {

            if (gamepad1.left_bumper && !pressed) servoNum -= 1;
            if (gamepad1.right_bumper && !pressed) servoNum += 1;
            while (servoNum < 0) servoNum += servos.length;
            while (servoNum > servos.length - 1) servoNum -= servos.length;

            if (!stringToServo().equals(previousServo)){
                targetServo = stringToServo();
                targetServo.scaleRange(0, 1);
                targetServo.setDirection(Servo.Direction.FORWARD);
            }

            if (gamepad1.dpad_up && !pressed) {
                servoPosition += 0.05;
            }
            if (gamepad1.dpad_down && !pressed) {
                servoPosition -= 0.05;
            }
            if (gamepad1.dpad_right && !pressed) {
                servoPosition += 0.01;
            }
            if (gamepad1.dpad_left && !pressed) {
                servoPosition -= 0.01;
            }
            if (gamepad1.y) servoPosition = 1;
            if (gamepad1.x) servoPosition = 0;

            if (gamepad1.left_bumper || gamepad1.right_bumper || gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_up || gamepad1.dpad_down) pressed = true;
            else pressed = false;

            servoPosition = Math.max(servoPosition, 0);
            servoPosition = Math.min(servoPosition, 1);
            if (gamepad1.a) update = true;
            if (gamepad1.b) update = false;
            if (update) {
                targetServo.setPosition(servoPosition);
            }

            telemetry.addData("Servo", servos[servoNum]);
            telemetry.addData("Servo Position", servoPosition);
            telemetry.addData("Update", update);
            telemetry.update();
        }
    }

    public Servo stringToServo() {
        switch(servos[servoNum]) {
            case "horizontal extender":
                return drive.horizontalExtender;
            case "left foundation mover":
                return drive.foundationMoverLeft;
            case "right foundation mover":
                return drive.foundationMoverRight;
            case "front skystone arm":
                return drive.skystoneArmFront;
            case "back skystone arm":
                return drive.skystoneArmBack;
            case "front skystone grabber":
                return drive.skystoneGrabberFront;
            case "back skystone grabber":
                return drive.skystoneGrabberBack;
            case "capstone arm":
                return drive.capstoneArm;
            case "parking arm":
                return drive.parkingArm;
            default:
                return drive.parkingArm;
        }
    }
}

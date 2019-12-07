package org.firstinspires.ftc.teamcode.yoda_code;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class YodaMecanumDrive extends SampleMecanumDriveREVOptimized {
    public ExpansionHubEx hub2;
    public ExpansionHubMotor verticalExtender;
    public Servo horizontalExtender;
    public Servo foundationMoverLeft, foundationMoverRight;
    public Servo skystoneGrabberFront, skystoneArmFront, skystoneGrabberBack, skystoneArmBack;
    public Servo capstoneArm, intakeGrabber;
    public Rev2mDistanceSensor frontLeftDistance, frontRightDistance, rightDistance;
    public ModernRoboticsI2cRangeSensor backDistance, frontDistance;

    public DcMotor leftEncoder, rightEncoder, frontEncoder;
    ElapsedTime global_timer;
    String last_tag_for_logging;

    public YodaMecanumDrive(HardwareMap hardwareMap) {
        super(hardwareMap);

        global_timer = new ElapsedTime();
        last_tag_for_logging = "";

        hub2 = hardwareMap.get(ExpansionHubEx.class, "Secondary Hub id: 2");

        verticalExtender = hardwareMap.get(ExpansionHubMotor.class, "vertical extender");

        horizontalExtender = hardwareMap.get(Servo.class, "horizontal extender");
        foundationMoverLeft = hardwareMap.get(Servo.class, "left foundation mover");
        foundationMoverRight = hardwareMap.get(Servo.class, "right foundation mover");

        skystoneGrabberFront = hardwareMap.get(Servo.class, "front skystone grabber");
        skystoneGrabberBack = hardwareMap.get(Servo.class, "back skystone grabber");
        skystoneArmFront = hardwareMap.get(Servo.class, "front skystone arm");
        skystoneArmBack = hardwareMap.get(Servo.class, "back skystone arm");
        capstoneArm = hardwareMap.get(Servo.class, "capstone arm");
        intakeGrabber = hardwareMap.get(Servo.class, "intake grabber");

        frontLeftDistance = hardwareMap.get(Rev2mDistanceSensor.class, "front left distance");
        frontRightDistance = hardwareMap.get(Rev2mDistanceSensor.class, "front right distance");
        rightDistance = hardwareMap.get(Rev2mDistanceSensor.class, "right distance");
        backDistance = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "back distance");
        frontDistance = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "front distance");

        verticalExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalExtender.setTargetPosition(0);
        verticalExtender.setPower(1);
        verticalExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalExtender.setTargetPositionTolerance(0);

        foundationMoverRight.setDirection(Servo.Direction.REVERSE);
        foundationMoverLeft.setDirection(Servo.Direction.REVERSE);
        capstoneArm.setDirection(Servo.Direction.REVERSE);
        skystoneArmFront.setDirection(Servo.Direction.REVERSE);
        skystoneGrabberFront.setDirection(Servo.Direction.REVERSE);
        horizontalExtender.setDirection(Servo.Direction.REVERSE);
        intakeGrabber.setDirection(Servo.Direction.REVERSE);

        //setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        leftEncoder = hardwareMap.dcMotor.get("leftEncoder");
        rightEncoder = hardwareMap.dcMotor.get("rightEncoder");
        frontEncoder = hardwareMap.dcMotor.get("frontEncoder");
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        rightEncoder.setDirection(DcMotor.Direction.REVERSE);
        frontEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        horizontalExtender.scaleRange(1 - 0.37, 1 - 0.13); // actually limit 0.37

        skystoneArmFront.scaleRange(1 - 0.33, 1);
        skystoneArmBack.scaleRange(0, 0.33);

        foundationMoverLeft.scaleRange(0.4, 1);
        foundationMoverRight.scaleRange(0.5, 1);
    }

    public void resetTimer() {
        global_timer.reset();
    }

    public void log(String message) {
        Log.i("Yoda","(" + global_timer.time(TimeUnit.MILLISECONDS) + "ms)"+last_tag_for_logging + " | " + message);
    }

    public void setLogTag(String tag) {
        last_tag_for_logging = tag;
    }

    public void strafeRight(double inches) {
        log("strafeRight:" + inches);
        followTrajectorySync(this.trajectoryBuilder()
                .strafeRight(inches)
                .build());
    }

    public void strafeLeft(double inches) {
        log("strafeLeft:" + inches);
        followTrajectorySync(this.trajectoryBuilder()
                .strafeLeft(inches)
                .build());
    }

    public void forward(double inches) {
        log("forward:" + inches);
        followTrajectorySync(this.trajectoryBuilder()
                .forward(inches)
                .build());
    }

    public void back(double inches) {
        log("back:" + inches);
        followTrajectorySync(this.trajectoryBuilder()
                .back(inches)
                .build());
    }

    public void turnToRadians(double angle) {
        turnToRadians(angle, getRawExternalHeading());
    }
    protected void turnToRadians(double angle, double currentAngle) {
        log("turnToRadians: angle" + Math.toDegrees(angle) + ", currentAngle:" + Math.toDegrees(currentAngle));
        angle = Math.toDegrees(angle) - Math.toDegrees(currentAngle);
        while (angle < -180) {
            angle += 360;
        }
        while (angle > 180) {
            angle -= 360;
        }
        log("turnToRadians: actual turning angle: " + angle);
        turnSync(Math.toRadians(angle));
    }

    public void setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (ExpansionHubMotor motor : getMotors()) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public List<Double> scaleDown(double a, double b, double c, double d, double max) {
        double biggestNumber = Math.abs(a);
        if (biggestNumber < Math.abs(b)) biggestNumber = Math.abs(b);
        if (biggestNumber < Math.abs(c)) biggestNumber = Math.abs(c);
        if (biggestNumber < Math.abs(d)) biggestNumber = Math.abs(d);

        if (Math.abs(biggestNumber) > max) {
            a = a / Math.abs(biggestNumber) * max;
            b = b / Math.abs(biggestNumber) * max;
            c = c / Math.abs(biggestNumber) * max;
            d = d / Math.abs(biggestNumber) * max;
        }
        return Arrays.asList(a, b, c, d);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted()
                && isBusy()
                && (opMode == null || !opMode.isStopRequested())) {
            update();
        }
    }

    public double keepBetweenMaxMin(double number, double max, double min) {
        return Math.min(Math.max(number, min), max);
    }

    public double getAngleToFront(Telemetry telemetry) {
        double adjacent =  10 + 1/16; // Distance between sensors, in.
        double right = frontRightDistance.getDistance(DistanceUnit.INCH);
        double left = frontLeftDistance.getDistance(DistanceUnit.INCH);

        double opposite = right - left;
        telemetry.addData("Distance", "Left %.2f, Right %.2f", left, right);
        if (left > 200 || right > 200) return 0;

        double angle = Math.atan(Math.abs(opposite) / adjacent);

        if (opposite < 0) angle = -angle;

        telemetry.addData("Angle", angle);
        return angle;
    }

    public double getRightDistance() {
        return rightDistance.getDistance(DistanceUnit.INCH);
    }

    public double getBackDistance() {

        double distance = backDistance.getDistance(DistanceUnit.INCH);
        if (distance > 1000) {
            //something is wrong
            return -1; // todo, add log
        }
        // 3.5 is sensor distance to robot border
        return distance - 3.5;
    }

    public double getFrontDistance() {
        double distance = frontDistance.getDistance(DistanceUnit.INCH);
        if (distance > 1000) {
            //something is wrong
            return -1; // todo, add log
        }
        // 1.5 is sensor distance to robot border
        return distance - 1.5;
    }

    public void resetServos() {
        // Skystone Arms
        skystoneArmFront.setPosition(0);
        skystoneArmBack.setPosition(0);
        skystoneGrabberFront.setPosition(0);
        skystoneGrabberBack.setPosition(0);

        // Foundation Movers
        foundationMoverLeft.setPosition(0);
        foundationMoverRight.setPosition(0);

        // Capstone Arm
        capstoneArm.setPosition(0.29);

        // Intake Grabber
//        intakeGrabber.setPosition(0.6);

        // Horizontal Extender
//        horizontalExtender.setPosition(0);
    }
}

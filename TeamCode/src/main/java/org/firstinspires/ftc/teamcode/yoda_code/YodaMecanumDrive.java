package org.firstinspires.ftc.teamcode.yoda_code;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.Arrays;
import java.util.List;

@Config
public class YodaMecanumDrive extends SampleMecanumDriveREVOptimized {
    public ExpansionHubEx hub2;
    public ExpansionHubMotor verticalExtender, parkingTape;
    public Servo horizontalExtender;
    public Servo foundationMoverLeft, foundationMoverRight;
    public Servo skystoneGrabberFront, skystoneArmFront, skystoneGrabberBack, skystoneArmBack;
    public Servo capstoneArm, intakeGrabber, parkingArm;
    public Rev2mDistanceSensor frontLeftDistance, frontRightDistance;
    public Rev2mDistanceSensor rightDistance;
//    public ModernRoboticsI2cRangeSensor backDistance, frontDistance;
    public ModernRoboticsI2cRangeSensor leftDistance;
    public ExpansionHubMotor leftEncoder, rightEncoder, frontEncoder;
    public RevBlinkinLedDriver led;
    private RevBlinkinLedDriver.BlinkinPattern lastPattern;
    public static double leftSensorToWall = 0;
    public static double sensorXOffset = 0;
    public static double sensorYOffset = -6 - 7.0/8.0;
    public static double middleToLeft = 8.214;
    private double newZeroIMU = 0;

//    ElapsedTime global_timer;
//    String last_tag_for_logging;

    public YodaMecanumDrive(HardwareMap hardwareMap) {
        super(hardwareMap);

        global_timer = new ElapsedTime();
        latency_timer = new ElapsedTime();
//        last_tag_for_logging = "";

        hub2 = hardwareMap.get(ExpansionHubEx.class, "Secondary Hub id: 2");

        leftEncoder = hardwareMap.get(ExpansionHubMotor.class, "leftEncoder");
        rightEncoder = hardwareMap.get(ExpansionHubMotor.class, "rightEncoder");
        frontEncoder = hardwareMap.get(ExpansionHubMotor.class, "frontEncoder");
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        frontEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        parkingTape = leftEncoder;
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
        parkingArm = hardwareMap.get(Servo.class, "parking arm");

        frontLeftDistance = hardwareMap.get(Rev2mDistanceSensor.class, "front left distance");
        frontRightDistance = hardwareMap.get(Rev2mDistanceSensor.class, "front right distance");
        rightDistance = hardwareMap.get(Rev2mDistanceSensor.class, "right distance");
        leftDistance = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "left distance");
//        backDistance = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "back distance");
//        frontDistance = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "front distance");

        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");

        verticalExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalExtender.setTargetPosition(0);
        verticalExtender.setPower(1);
        verticalExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalExtender.setTargetPositionTolerance(0);

        skystoneArmFront.setDirection(Servo.Direction.REVERSE);
        skystoneGrabberFront.setDirection(Servo.Direction.REVERSE);
        horizontalExtender.setDirection(Servo.Direction.REVERSE);
        capstoneArm.setDirection(Servo.Direction.REVERSE);

        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        horizontalExtender.scaleRange(1 - 0.37, 1 - 0.13); // actually limit 0.37

        skystoneArmFront.scaleRange(0.7, 1);
        skystoneArmBack.scaleRange(0, 0.28);

        parkingArm.scaleRange(0, 0.45);

        intakeGrabber.scaleRange(0, 0.45);

        foundationMoverLeft.scaleRange(0.02, 0.88);
        foundationMoverRight.scaleRange(0.1, 0.79);
    }

    public void resetTimer() {
        global_timer.reset();
    }
    public void resetLatencyTimer() {
        latency_timer.reset();
    }

//    public void log(String message) {
//        Log.i("Yoda","(" + global_timer.time(TimeUnit.MILLISECONDS) + "ms)"+last_tag_for_logging + " | " + message);
//    }

//    public void setLogTag(String tag) {
//        last_tag_for_logging = tag;
//    }

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

    public void strafeTo(Vector2d position) {
        followTrajectorySync(trajectoryBuilder()
                .strafeTo(position)
                .build());
    }

    public void turnToRadians(double angle) {
        turnToRadians(angle, getRawExternalHeading());
    }
    public void turnToRadians(double angle, double currentAngle) {
        log("turnToRadians: angle" + Math.toDegrees(angle) + ", currentAngle:" + Math.toDegrees(currentAngle));
        angle = Math.toDegrees(angle) - Math.toDegrees(currentAngle);
        angle = convertAngle180(angle);
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

    public int    minMax(int    number, int    min, int    max) { return Math.min(Math.max(number, min), max);}
    public long   minMax(long   number, long   min, long   max) { return Math.min(Math.max(number, min), max);}
    public float  minMax(float  number, float  min, float  max) { return Math.min(Math.max(number, min), max);}
    public double minMax(double number, double min, double max) { return Math.min(Math.max(number, min), max);}

//    public double getAngleToFront(Telemetry telemetry) {
//        double adjacent =  10 + 1.0/16.0; // Distance between sensors, in.
//        double right = frontRightDistance.getDistance(DistanceUnit.INCH);
//        double left = frontLeftDistance.getDistance(DistanceUnit.INCH);
//
//        double opposite = right - left;
//        telemetry.addData("Distance", "Left %.2f, Right %.2f", left, right);
//        if (left > 200 || right > 200) return 0;
//
//        double angle = Math.atan(Math.abs(opposite) / adjacent);
//
//        if (opposite < 0) angle = -angle;
//
//        telemetry.addData("Angle", angle);
//        return angle;
//    }
//
//    public double getGreatestBottomFrontDistance() { return Math.max(getFrontLeftDistance(), getFrontRightDistance());}
//
    public double getFrontLeftDistance() { return frontLeftDistance.getDistance(DistanceUnit.INCH);}
    public double getFrontRightDistance() { return frontRightDistance.getDistance(DistanceUnit.INCH);}
    public double getRightDistance() {
        return rightDistance.getDistance(DistanceUnit.INCH);
    }
    public double getLeftDistance() {
        return leftDistance.getDistance(DistanceUnit.INCH) - 2; // 2" is the sensor to wall
    }

    public double getBackDistance() {
//
//        double distance = backDistance.getDistance(DistanceUnit.INCH);
//        if (distance > 1000) {
//            //something is wrong
//            return -1; // todo, add log
//        }
//        // 3.5 is sensor distance to robot border
//        return distance - 3.5;
        return 0;
    }

    public double getFrontDistance() {
//        double distance = frontDistance.getDistance(DistanceUnit.INCH);
//        if (distance > 1000) {
//            //something is wrong
//            return -1; // todo, add log
//        }
//        // 1.5 is sensor distance to robot border
//        return distance - 1.5;
        return 0;
    }

    public void resetInitServos() {
        // Skystone Arms
        skystoneArmFront.setPosition(0);
        skystoneArmBack.setPosition(0);
        skystoneGrabberFront.setPosition(0);
        skystoneGrabberBack.setPosition(0);

        // Foundation Movers
        foundationMoverLeft.setPosition(0);
        foundationMoverRight.setPosition(0);

        // Parking Arm
        parkingArm.setPosition(0);

        // Capstone Arm
        capstoneArm.setPosition(0);
    }

    public void resetAllServos() {
        resetInitServos();

        // Intake Grabber
        intakeGrabber.setPosition(0);
    }

    public void setMotorNoEncoder() {
        for (ExpansionHubMotor motor : getMotors()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void turnLedOff() {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    public void setLed(RevBlinkinLedDriver.BlinkinPattern pattern) {
        if (lastPattern != pattern) led.setPattern(pattern);
        lastPattern = pattern;
    }

    @Override
    public double getCalculatedY(double startingY) {
        double heading = getHeading();
        double leftDist = getLeftDistance();
        double negativeMultiplier = startingY < 0 ? -1 : 1;
//        startingY += middleToLeft * negativeMultiplier;
        if ((60 < Math.toDegrees(heading) && Math.toDegrees(heading) < 360 - 60) || leftDist > 300 || isNaN(leftDist)) return getY();

        double robotLeftToWall = (leftDist + Math.abs(sensorYOffset)) * Math.cos(heading) + sensorXOffset * Math.sin(heading);

        return startingY - (robotLeftToWall * negativeMultiplier);
    }

    public void resetLeftSensorToWall(double startingY, double extra) {
/*        leftSensorToWall = getLeftDistance() + Math.abs(sensorYOffset);
        if (startingY >= 0) leftSensorToWall += extra;
        else if (startingY < 0) leftSensorToWall -= extra;*/
    }

    public double getX() { return getPoseEstimate().getX();}
    public double getY() { return getPoseEstimate().getY(); }
    public double getHeading() { return getPoseEstimate().getHeading();}

    public double getIMUHeading() {
        double heading = getRawExternalHeading() + newZeroIMU;
        while (heading > 2 * Math.PI) heading -= 2 * Math.PI; // 2 * Math.PI radians is equal to 360 degrees
        return heading;
    }
    public void resetIMUHeading() {
        newZeroIMU = getRawExternalHeading();
    }

    public double pow(double base, double exponent) {
        double result = Math.pow(Math.abs(base), exponent);
        if (base < 0) result *= -1;
        return result;
    }

    public double getAngleToTurn(double currentHeading, double targetHeading) {
        log("Current Heading: " + Math.toDegrees(currentHeading));
        currentHeading = convertAngle180(Math.toDegrees(currentHeading));
        log("Current Heading: " + currentHeading);

        log("Target Heading: " + Math.toDegrees(targetHeading));
        targetHeading = convertAngle180(Math.toDegrees(targetHeading));
        log("Target Heading: " + targetHeading);
        log("Angle To Turn: " + (targetHeading - currentHeading));
        return Math.toRadians(targetHeading - currentHeading);
    }

    public double convertAngle180(double degrees) { // converts to -180 to 180
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }

    public double convertAngle360(double degrees) { // converts to 0 to 360
        while (degrees >= 360) degrees -= 360;
        while (degrees < 0) degrees += 360;
        return degrees;
    }

    public boolean isNaN(double number) {
        return number != number;
    }
}

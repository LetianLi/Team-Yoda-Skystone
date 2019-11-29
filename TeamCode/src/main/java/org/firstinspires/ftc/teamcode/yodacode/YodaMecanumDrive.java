package org.firstinspires.ftc.teamcode.yodacode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.Arrays;
import java.util.List;

public class YodaMecanumDrive extends SampleMecanumDriveREVOptimized {
    public ExpansionHubEx hub2;
    public ExpansionHubMotor verticalExtender;
    public Servo horizontalExtender;
    public Servo foundationMoverLeft, foundationMoverRight;
    public Servo skystoneGrabberFront, skystoneArmFront, skystoneGrabberBack, skystoneArmBack;
    public Servo capstoneArm, intakeGrabber;
    public Rev2mDistanceSensor frontLeftDistance, frontRightDistance;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private ElapsedTime global_timer;
    public DcMotor leftEncoder, rightEncoder, frontEncoder;

    public YodaMecanumDrive(HardwareMap hardwareMap) {
        super(hardwareMap);

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

        verticalExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalExtender.setTargetPosition(0);
        verticalExtender.setPower(1);
        verticalExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        foundationMoverRight.setDirection(Servo.Direction.REVERSE);
        capstoneArm.setDirection(Servo.Direction.REVERSE);
        skystoneArmFront.setDirection(Servo.Direction.REVERSE);
        skystoneGrabberFront.setDirection(Servo.Direction.REVERSE);
        horizontalExtender.setDirection(Servo.Direction.REVERSE);
        intakeGrabber.setDirection(Servo.Direction.REVERSE);

        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

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

    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void strafeRight(double inches) {
        followTrajectorySync(this.trajectoryBuilder()
                .strafeRight(inches)
                .build());
    }

    public void strafeLeft(double inches) {
        followTrajectorySync(this.trajectoryBuilder()
                .strafeLeft(inches)
                .build());
    }

    public void forward(double inches) {
        followTrajectorySync(this.trajectoryBuilder()
                .forward(inches)
                .build());
    }

    public void back(double inches) {
        followTrajectorySync(this.trajectoryBuilder()
                .back(inches)
                .build());
    }

    /*public double getFrontDistance(DistanceUnit unit) {
        return frontDistance.getDistance(unit);
    }*/

    public void setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (ExpansionHubMotor motor : getMotors()) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public List<Double> scaleDown(double a, double b, double c, double d, double max) {
        double biggestNumber = a;
        if (biggestNumber < b) biggestNumber = b;
        if (biggestNumber < c) biggestNumber = c;
        if (biggestNumber < d) biggestNumber = d;

        if (biggestNumber > max) {
            a = a / biggestNumber * max;
            b = b / biggestNumber * max;
            c = c / biggestNumber * max;
            d = d / biggestNumber * max;
        }
        return Arrays.asList(a, b, c, d);
    }

    public double getAngleToFront() {
        double adjacent =  10 + 1/16; // Distance between sensors, in.
        double right = frontRightDistance.getDistance(DistanceUnit.INCH);
        double left = frontLeftDistance.getDistance(DistanceUnit.INCH);
        double opposite = right - left;

        if (left > 150 || right > 150) return 0;

        double angle = Math.atan(Math.abs(opposite) / adjacent);

        if (opposite < 0) angle = -angle;

        return angle;
    }
}

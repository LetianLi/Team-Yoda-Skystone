package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.teamcode.SKYSTONEVuforiaDetector;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

public class YodaMecanumDrive extends SampleMecanumDriveREVOptimized {
    public ExpansionHubEx hub2;
    public ExpansionHubMotor verticalExtender;
    public Servo horizontalExtender;
    public Servo foundationMoverLeft, foundationMoverRight;
    public Servo skystoneGrabberFront, skystoneArmFront, skystoneGrabberBack, skystoneArmBack;
    public Servo capstoneArm, intakeGrabber;
    public Rev2mDistanceSensor frontDistance;
    public SKYSTONEVuforiaDetector vuforiaDetector;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private ElapsedTime global_timer;

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

        frontDistance = hardwareMap.get(Rev2mDistanceSensor.class, "front distance");

        verticalExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalExtender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        foundationMoverRight.setDirection(Servo.Direction.REVERSE);
        capstoneArm.setDirection(Servo.Direction.REVERSE);


    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void activateImageDetector() {
        vuforiaDetector = new SKYSTONEVuforiaDetector();
        vuforiaDetector.setTelemetry(this.telemetry);
        vuforiaDetector.initialize(hardwareMap);
        vuforiaDetector.activateDetection();
    }

    public void deactiveImageDetector() {
        if (vuforiaDetector != null) {
            vuforiaDetector.deactivateDetection();
        }
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

    public double getFrontDistance(DistanceUnit unit) {
        return frontDistance.getDistance(unit);
    }
}

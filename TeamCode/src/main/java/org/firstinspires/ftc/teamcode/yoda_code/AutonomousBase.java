package org.firstinspires.ftc.teamcode.yoda_code;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yoda_enum.SkystonePos;
import org.firstinspires.ftc.teamcode.yoda_enum.TeamColor;

public abstract class AutonomousBase extends LinearOpMode {

    public YodaMecanumDrive drive;
    protected ElapsedTime op_timer;
    ElapsedTime timer_for_led;
    ElapsedTime time_since_detection_start;
    public ElapsedTime auto_timer;
    protected StrategistBase strategist;
    protected TeamColor teamColor = TeamColor.UNKNOWN;
    protected boolean askTeamColor = true;
    protected boolean detectStoneAndWait = true;

    private SkystonePos skystonePos = SkystonePos.UNKNOW;
    public OpencvDetector detector;
    private boolean bumpersPressed = false;

    protected void initialize() {
        telemetry.addData("Status", "Initializing, please wait");
        telemetry.update();

        detector = new OpencvDetector(hardwareMap);

        drive = new YodaMecanumDrive(hardwareMap);
        drive.setOpMode(this);

        op_timer = new ElapsedTime();
        timer_for_led = new ElapsedTime();
        time_since_detection_start = new ElapsedTime();
        auto_timer = new ElapsedTime();
        drive.resetInitServos();
        drive.setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (askTeamColor) {
            askTeamColor();
        }

        if (teamColor == TeamColor.BLUE) {
            strategist = new BlueStrategist(drive, op_timer, this);
        } else if (teamColor == TeamColor.RED) {
            strategist = new RedStrategist(drive, op_timer, this);
        } else {
            telemetry.addData("Error", "Started the robot before picking team");
            telemetry.update();
            requestOpModeStop();
            return;
        }
        drive.setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.log().add("Ready!");
        telemetry.update();
        if (detectStoneAndWait) detectStoneWhileWaiting();
        drive.resetTimer();
        drive.setLogTag("initialize");
        drive.log("Skystone position:" + getSkystonePos());
        drive.log("Team color:" + teamColor);
    }

    protected void detectStoneWhileWaiting() {
        String previousDetectorValues = "null";
        timer_for_led.reset();
        time_since_detection_start.reset();
        if (teamColor == TeamColor.RED) drive.setLed(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
        else if (teamColor == TeamColor.BLUE) drive.setLed(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);

        while (!isStarted() && !isStopRequested()) {
            movePoints();
            telemetry.addData("Status", "Initialized. Wait to start");
            telemetry.addData("TeamColor", teamColor.toString());
            telemetry.addData("Values", detector.getValues());
            if (!detector.getValues().equals(previousDetectorValues)) {
                timer_for_led.reset();
                drive.log("Detector values change from (" + previousDetectorValues + ") to (" + detector.getValues() + ")");
                previousDetectorValues = detector.getValues();
            }
            skystonePos = detector.getSkystonePos();
            telemetry.addData("Skystone Pos", skystonePos);
            telemetry.addData("Front Distance", drive.getFrontDistance());
            telemetry.addData("Back Distance", drive.getBackDistance());
            telemetry.addData("Left Distance", drive.getLeftDistance());
            telemetry.addData("Right Distance", drive.getRightDistance());
            telemetry.update();
            if (time_since_detection_start.seconds() > 2) {
                if (detector.getNumberOfSkystones() == 1) {
                    if (timer_for_led.seconds() <= 5) {
                        if (skystonePos == SkystonePos.LEFT)
                            drive.setLed(RevBlinkinLedDriver.BlinkinPattern.RED);
                        else if (skystonePos == SkystonePos.MIDDLE)
                            drive.setLed(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        else if (skystonePos == SkystonePos.RIGHT)
                            drive.setLed(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                    } else {
                        drive.setLed(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                    }
                } else {
                    if (teamColor == TeamColor.RED)
                        drive.setLed(RevBlinkinLedDriver.BlinkinPattern.CP2_LARSON_SCANNER);
                    else if (teamColor == TeamColor.BLUE)
                        drive.setLed(RevBlinkinLedDriver.BlinkinPattern.CP1_LARSON_SCANNER);
                }
            } else timer_for_led.reset();

            idle();
            sleep(50);
        }
        op_timer.reset();
        drive.setLed(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        drive.resetAllServos();
    }

    public SkystonePos getSkystonePos() {
        if (skystonePos != SkystonePos.UNKNOW) {
            return skystonePos;
        } else {
            return SkystonePos.MIDDLE; //default
        }
    }

    protected void askTeamColor() {
        drive.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        telemetry.log().clear();
        telemetry.log().add("Please pick team color?");
        telemetry.log().add("Press (X) for BLUE, (B) for RED");
        telemetry.update();

        while (!isStopRequested()) {
            if (gamepad1.x || gamepad2.x) {
                teamColor = TeamColor.BLUE;
                break;
            } else if (gamepad1.b || gamepad2.b) {
                teamColor = TeamColor.RED;
                break;
            }
            telemetry.addData("Status", "Picking up team");
            if (isStarted()) {
                telemetry.addData("Error", "Started the robot before picking team");
                telemetry.update();
                requestOpModeStop();
                break;
            }

            telemetry.update();
            idle();
        }
        telemetry.log().clear();
    }

    // return y error
    protected double logError(String context, double x, double y) {
        drive.update();
        Pose2d currentPos = drive.getPoseEstimate();
        double imu = Math.toDegrees(drive.getRawExternalHeading());

        drive.log(context
                + String.format("\nX expected:%.3f, actual: %.3f, error: %.3f",x, currentPos.getX(),  (currentPos.getX() - x))
                + String.format("\nY expected:%.3f, actual: %.3f, error: %.3f",y, currentPos.getY(),  (currentPos.getY() - y))
                + String.format("\nHeading:%.3f", Math.toDegrees(currentPos.getHeading()))
                + String.format("\nimu:%.3f", imu));
        return currentPos.getY() - y;
    }

    public void deactivateDetector() {
        detector.deactivate();
    }

    public void movePoints() {
        if (detector != null) {
            double xMove = 0;
            double yMove = 0;
            if (!bumpersPressed) {
                if (gamepad1.right_bumper || gamepad2.right_bumper) {
                    detector.incrementMovablePos();
                    bumpersPressed = true;
                }
                if (gamepad1.left_bumper || gamepad2.left_bumper) {
                    detector.decrementMovablePos();
                    bumpersPressed = true;
                }
            }
            else if (!(gamepad1.left_bumper || gamepad2.left_bumper || gamepad1.right_bumper || gamepad2.right_bumper)) {
                bumpersPressed = false;
            }

            if (gamepad1.a || gamepad2.a) detector.stopMovable();

            if (gamepad1.dpad_left || gamepad2.dpad_left) xMove += -1;
            if (gamepad1.dpad_right || gamepad2.dpad_right) xMove += 1;

            if (gamepad1.dpad_down || gamepad2.dpad_down) yMove -= -1;
            if (gamepad1.dpad_up || gamepad2.dpad_up) yMove -= 1;

            detector.movePoint(xMove, yMove);
        }
    }
}

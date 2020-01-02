package org.firstinspires.ftc.teamcode.yoda_code;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yoda_enum.SkystonePos;
import org.firstinspires.ftc.teamcode.yoda_enum.TeamColor;

public abstract class AutonomousBase extends LinearOpMode {

    public YodaMecanumDrive drive;
    ElapsedTime op_timer;
    ElapsedTime timer_for_led;
    protected StrategistBase strategist;
    protected TeamColor teamColor = TeamColor.UNKNOWN;
    protected boolean askTeamColor = true;

    private SkystonePos skystonePos = SkystonePos.UNKNOW;
    private OpencvDetector detector;

    protected void initialize() {
        telemetry.addData("Status", "Initializing, please wait");
        telemetry.update();

        detector = new OpencvDetector(hardwareMap);

        drive = new YodaMecanumDrive(hardwareMap);
        drive.setOpMode(this);

        op_timer = new ElapsedTime();
        timer_for_led = new ElapsedTime();
        drive.resetServos();
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
        detectStoneWhileWaiting();
        drive.resetTimer();
        drive.setLogTag("initialize");
        drive.log("Skystone position:" + getSkystonePos());
        drive.log("Team color:" + teamColor);
    }

    protected void detectStoneWhileWaiting() {
        String previousDetectorValues = "";
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Initialized. Wait to start");
            telemetry.addData("TeamColor", teamColor.toString());
            telemetry.addData("Values", detector.getValues());
            if (detector.getValues() != previousDetectorValues) {
                timer_for_led.reset();
                previousDetectorValues = detector.getValues();
            }
            skystonePos = detector.getSkystonePos();
            telemetry.addData("Skystone Pos", skystonePos);
            telemetry.addData("Front Distance", drive.getFrontDistance());
            telemetry.addData("Back Distance", drive.getBackDistance());
            telemetry.addData("Left Distance", drive.getLeftDistance());
            telemetry.addData("Right Distance", drive.getRightDistance());
            telemetry.update();

            if (detector.getNumberOfSkystones() == 1) {
                if (timer_for_led.seconds() <= 3) {
                    if (skystonePos == SkystonePos.LEFT)
                        drive.setLed(RevBlinkinLedDriver.BlinkinPattern.RED);
                    else if (skystonePos == SkystonePos.MIDDLE)
                        drive.setLed(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    else if (skystonePos == SkystonePos.RIGHT)
                        drive.setLed(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                } else {
                    drive.setLed(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                }
            }
            else {
                if (teamColor == TeamColor.RED) drive.setLed(RevBlinkinLedDriver.BlinkinPattern.CP2_LARSON_SCANNER);
                else if (teamColor == TeamColor.BLUE) drive.setLed(RevBlinkinLedDriver.BlinkinPattern.CP1_LARSON_SCANNER);
            }

            idle();
            sleep(50);
        }
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
    public void deactivateDetector() {
        detector.deactivate();
    }
}

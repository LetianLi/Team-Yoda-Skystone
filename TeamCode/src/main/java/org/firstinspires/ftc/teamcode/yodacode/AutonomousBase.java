package org.firstinspires.ftc.teamcode.yodacode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class AutonomousBase extends LinearOpMode {

    public YodaMecanumDrive drive;
    ElapsedTime op_timer;
    protected StrategistBase strategist;
    protected TeamColor teamColor = TeamColor.UNKNOWN;
    protected boolean useImageDetector = true;
    protected boolean askTeamColor = true;

    protected void initialize() {
        telemetry.addData("Status", "Initializing, please wait");
        telemetry.update();

        drive = new YodaMecanumDrive(hardwareMap);
        drive.setTelemetry(telemetry);
        drive.setOpMode(this);

        op_timer = new ElapsedTime();
        drive.resetServos();

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

        telemetry.log().add("Ready!");
        telemetry.update();
        waitForStartPrintMsg();
    }

    protected void waitForStartPrintMsg() {
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Initialized. Wait to start");
            telemetry.addData("TeamColor", teamColor.toString());
            telemetry.update();
            idle();
            sleep(100);
        }
    }

    protected void askTeamColor() {
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
}

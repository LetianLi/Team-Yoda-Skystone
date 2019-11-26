package org.firstinspires.ftc.teamcode.yodacode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@TeleOp(group = "test")
public class MotorTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        YodaMecanumDrive drive = new YodaMecanumDrive(hardwareMap);

        waitForStart();

        sleep(1000);
        telemetry.addData("Motor", "leftFront");
        telemetry.update();
        drive.setMotorPowers(1, 0, 0, 0);
        sleep(1000);

        telemetry.addData("Motor", "leftRear");
        telemetry.update();
        drive.setMotorPowers(0, 1, 0, 0);
        sleep(1000);
        telemetry.addData("Motor", "rightRear");
        telemetry.update();
        drive.setMotorPowers(0, 0, 1, 0);
        sleep(1000);
        telemetry.addData("Motor", "rightFront");
        telemetry.update();
        drive.setMotorPowers(0, 0, 0, 1);
        sleep(1000);

        drive.setMotorPowers(0, 0, 0, 0);
        telemetry.addData("Encoders", drive.getWheelPositions());
        telemetry.update();
        sleep(5000);
    }
}


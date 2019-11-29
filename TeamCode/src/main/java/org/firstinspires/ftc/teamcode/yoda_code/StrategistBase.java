package org.firstinspires.ftc.teamcode.yoda_code;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yoda_enum.ArmSide;
import org.firstinspires.ftc.teamcode.yoda_enum.ArmStage;

public abstract class StrategistBase {
    protected YodaMecanumDrive drive;
    protected ElapsedTime op_timer;
    protected AutonomousBase opMode;

    public StrategistBase(
            YodaMecanumDrive drive,
            ElapsedTime op_timer,
            AutonomousBase opMode) {
        this.drive = drive;
        this.op_timer = op_timer;
        this.opMode = opMode;
    }

    public abstract void GrabSkyStone();

    public abstract void moveAndDropSkystoneOnFoundation();

    public abstract void fromFoundationToPark();

    public abstract void DoubleSkystoneDelivery();

    public abstract void turnAndMoveFoundationAndPark();

    protected void moveSkystoneArms(ArmSide side, ArmStage stage) {
        Servo targetArm = (side == ArmSide.FRONT) ? drive.skystoneArmFront : drive.skystoneArmBack;
        Servo targetGrabber = (side == ArmSide.FRONT) ? drive.skystoneGrabberFront : drive.skystoneGrabberBack;
        Servo secondaryArm = (side == ArmSide.BACK) ? drive.skystoneArmFront : drive.skystoneArmBack;
        Servo secondaryGrabber = (side == ArmSide.BACK) ? drive.skystoneGrabberFront : drive.skystoneGrabberBack;

        switch (stage) {
            case PREPARE:
                secondaryArm.setPosition(0);
                secondaryGrabber.setPosition(0);

                targetArm.setPosition(0);
                targetGrabber.setPosition(1);
                break;
            case GRAB:
                targetArm.setPosition(1);
                opMode.sleep(500);
                targetGrabber.setPosition(0);
                opMode.sleep(600);
                targetArm.setPosition(0);
                break;
            case DROP:
                targetArm.setPosition(1 - 0.15);
                opMode.sleep(500);
                targetGrabber.setPosition(1);
                opMode.sleep(100);
                targetArm.setPosition(0);
                break;
        }
    }





    protected void turnTo(double angle) {
        angle -= Math.toDegrees(drive.getExternalHeading());
        while (angle < -180) {
            angle += 360;
        }
        while (angle > 180) {
            angle -= 360;
        }
        if (angle > 180) angle -= 360;
        drive.turnSync(Math.toRadians(angle));
    }
    protected void forward(double distance) {
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(distance)
                .build());
    }
    protected void strafeRight(double distance) {
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeRight(distance)
                .build());
    }
    protected void back(double distance) {
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .back(distance)
                .build());
    }
    protected void strafeLeft(double distance) {
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeLeft(distance)
                .build());
    }
}

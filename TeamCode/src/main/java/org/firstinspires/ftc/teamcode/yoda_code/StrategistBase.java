package org.firstinspires.ftc.teamcode.yoda_code;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yoda_enum.ArmSide;
import org.firstinspires.ftc.teamcode.yoda_enum.ArmStage;
import org.firstinspires.ftc.teamcode.yoda_enum.SkystonePos;

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
                opMode.sleep(100);
                targetGrabber.setPosition(0);
                break;
        }
    }

    protected void moveFoundationServos(double position) {
        drive.foundationMoverLeft.setPosition(position);
        drive.foundationMoverRight.setPosition(position);
    }



    public double getForwardOffset(SkystonePos skystonePos, double[] forwardOffsetsPerPos) {
        switch(skystonePos) {
            case LEFT:
                return forwardOffsetsPerPos[0];
            case MIDDLE:
                return forwardOffsetsPerPos[1];
            case RIGHT:
                return forwardOffsetsPerPos[2];
            case UNKNOW:
                return forwardOffsetsPerPos[1];
        }
        return forwardOffsetsPerPos[1];
    }

    protected void turnTo(double angle) {
        turnTo(angle, drive.getRawExternalHeading());
    }
    protected void turnTo(double angle, double currentAngle) {
        angle -= Math.toDegrees(currentAngle);
        while (angle < -180) {
            angle += 360;
        }
        while (angle > 180) {
            angle -= 360;
        }
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
    protected double getX() {
        return drive.getPoseEstimate().getX();
    }
    protected double getY() {
        return drive.getPoseEstimate().getY();
    }
    protected double getHeading() {
        return drive.getPoseEstimate().getHeading();
    }
    protected void updatePose() {
        drive.updatePoseEstimate();
    }
}

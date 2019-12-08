package org.firstinspires.ftc.teamcode.yoda_code;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    public void readyForManual() {
        drive.setLogTag("readyForManual");
        drive.horizontalExtender.setPosition(1);
        drive.intakeGrabber.setPosition(0.4);
        moveFoundationServos(0);
        drive.log("move horizontal extender out");
        opMode.sleep(3000);
    }

    public abstract void calculateDistance();

    public abstract void grabSkyStone();

    public abstract void moveAndDropSkystoneOnFoundation();

    public abstract void fromFoundationToPark();

    public abstract void goBackGrabDeliverSecondSkystone();

    public abstract void turnAndMoveFoundationAndPark();

    public abstract void moveFoundationBackAndPark();

    protected void moveSkystoneArms(ArmSide side, ArmStage stage) {
        Servo targetArm = (side == ArmSide.FRONT) ? drive.skystoneArmFront : drive.skystoneArmBack;
        Servo targetGrabber = (side == ArmSide.FRONT) ? drive.skystoneGrabberFront : drive.skystoneGrabberBack;
        // also control the other arm/grabber in prepare stage to be at stored position
        Servo theOtherArm = (side == ArmSide.BACK) ? drive.skystoneArmFront : drive.skystoneArmBack;
        Servo theOtherGrabber = (side == ArmSide.BACK) ? drive.skystoneGrabberFront : drive.skystoneGrabberBack;

        switch (stage) {
            case PREPARE:
                theOtherArm.setPosition(0);
                theOtherGrabber.setPosition(0);

                targetArm.setPosition(0);
                targetGrabber.setPosition(1); // open grabber
                break;
            case GRAB:
                targetArm.setPosition(1); // put down arm
                opMode.sleep(400);
                targetGrabber.setPosition(0); // grab it
                opMode.sleep(500);
                targetArm.setPosition(0); // put arm up. No need for sleep as next step is moving robot away
                break;
            case DROP:
                targetArm.setPosition(1 - 0.15); // This attempts to put arm down, but no wait before opening grabber, can be deleted?
                opMode.sleep(100);
                targetGrabber.setPosition(1); // open grabber
                opMode.sleep(200);
                targetArm.setPosition(0); // Put arm up
//                opMode.sleep(100); // Do we need this wait? can be deleted?
//                targetGrabber.setPosition(0); // close grabber
                break;
        }
    }

    protected void moveFoundationServos(double position) {
        drive.foundationMoverLeft.setPosition(position);
        drive.foundationMoverRight.setPosition(position);
    }

    public double moveRightToDistance(double distanceFromRight, boolean doLeftInCase, double maxMovementRight) {
        drive.setLogTag("moveRightToDistance");
        double currentDistance = drive.getRightDistance();
        drive.log("current distance right:" + currentDistance);
        if (currentDistance > distanceFromRight && currentDistance - distanceFromRight <= maxMovementRight) {
            drive.strafeRight(currentDistance - distanceFromRight);
        }
        else if (currentDistance - distanceFromRight >= maxMovementRight) {
            drive.strafeRight(maxMovementRight);
            return maxMovementRight;
        }
        else if (distanceFromRight > currentDistance && doLeftInCase) {
            drive.strafeLeft(distanceFromRight - currentDistance);
        }
        return currentDistance - distanceFromRight;
    }

    protected double getForwardOffset(SkystonePos skystonePos, double[] forwardOffsetsPerPos) {
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

package org.firstinspires.ftc.teamcode.yoda_code;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yoda_enum.ArmSide;
import org.firstinspires.ftc.teamcode.yoda_enum.ArmStage;
import org.firstinspires.ftc.teamcode.yoda_enum.SkystonePos;

public abstract class StrategistBase {
    public YodaMecanumDrive drive;
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

    public void resetSkystoneArms() {
        drive.skystoneArmFront.setPosition(0);
        drive.skystoneArmBack.setPosition(0);
        drive.skystoneGrabberFront.setPosition(0);
        drive.skystoneGrabberBack.setPosition(0);
    }

    public void moveSkystoneArms(ArmSide side, ArmStage stage) {
        Servo targetArm = (side == ArmSide.FRONT) ? drive.skystoneArmFront : drive.skystoneArmBack;
        Servo targetGrabber = (side == ArmSide.FRONT) ? drive.skystoneGrabberFront : drive.skystoneGrabberBack;
        // also control the other arm/grabber in prepare stage to be at stored position
        Servo theOtherArm = (side == ArmSide.BACK) ? drive.skystoneArmFront : drive.skystoneArmBack;
        Servo theOtherGrabber = (side == ArmSide.BACK) ? drive.skystoneGrabberFront : drive.skystoneGrabberBack;
        drive.log("Move Skystone Arms - " + stage.toString() + " - at " + drive.getPoseEstimate().toString());
        switch (stage) {
            case RESET:
                theOtherArm.setPosition(0);
                theOtherGrabber.setPosition(0);

                targetArm.setPosition(0);
                targetGrabber.setPosition(0);
                break;
            case OPENGRABBER:
                targetGrabber.setPosition(1); // open grabber
                break;
            case LOWERARM:
                targetArm.setPosition(1); // put down arm
                break;
            case CLOSEGRABBER:
                targetGrabber.setPosition(0);
                break;
            case RETRACTARM:
                targetArm.setPosition(0);
                break;
            case PREPDROP:
                targetArm.setPosition(0.4);
                break;
            case PREPARM:
                targetArm.setPosition(1); // was 0.8
                break;
//            case PREPGRAB:
//                targetGrabber.setPosition(0.4);
//                break;

            case GRAB:
                targetArm.setPosition(1);
                targetGrabber.setPosition(0); // grab it
                opMode.sleep(250);
                targetArm.setPosition(0); // put arm up. No need for sleep as next step is moving robot away
                opMode.sleep(50);
                break;
            case DROP:
                targetArm.setPosition(1 - 0.16); // This attempts to put arm down
                targetGrabber.setPosition(1); // open grabber
                opMode.sleep(200);
                targetArm.setPosition(0); // Put arm up
//                opMode.sleep(100); // Do we need this wait? can be deleted?
//                targetGrabber.setPosition(0); // close grabber
                break;
        }
    }

    public void moveFoundationServos(double position) {
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
            drive.strafeLeft(Math.abs(currentDistance - distanceFromRight));
        }
        return currentDistance - distanceFromRight;
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

    public double getX() { return drive.getX();}
    public double getY() { return drive.getY();}
    public double getHeading() { return drive.getHeading();}
    public void updatePose() { drive.updatePoseEstimate();}
}

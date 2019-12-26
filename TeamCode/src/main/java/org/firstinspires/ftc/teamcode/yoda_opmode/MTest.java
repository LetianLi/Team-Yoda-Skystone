package org.firstinspires.ftc.teamcode.yoda_opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.yoda_code.AutonomousBase;
import org.firstinspires.ftc.teamcode.yoda_enum.ArmSide;
import org.firstinspires.ftc.teamcode.yoda_enum.ArmStage;
import org.firstinspires.ftc.teamcode.yoda_enum.SkystonePos;
import org.firstinspires.ftc.teamcode.yoda_enum.TeamColor;

@Autonomous(group = "auto", name = "MTest")
public class MTest extends AutonomousBase {

    private double[] blueForwardOffsets = {0, 8, 16}; // left, middle, right
    private double[] redForwardOffsets  = {16, 8, 0}; // left middle, right
    private double forwardOffset = 0;
    private double negativeMultiplier = 1;

    private double stoneY = 32;
    private double foundationY = 31;

    private double[] armOrder;
    private double frontArm = -4.5;
    private double backArm = 7;

    private double thirdStoneOffset = blueForwardOffsets[0];

    private Pose2d buildingZone = new Pose2d(20, 41, 0);
    private Pose2d loadingZone = new Pose2d(-12, 41, 0);
    private Pose2d centerLine = new Pose2d(0, 41, 0);

    private Pose2d startingPos;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        if (isStopRequested()) return;
        if (strategist != null) {

            setUpVariables();
            strategist.moveSkystoneArms(getArmSide(armOrder[0]), ArmStage.PREPAREARM);
            strategist.moveSkystoneArms(getArmSide(armOrder[0]), ArmStage.OPENGRABBER);

            drive.setLogTag("main");
            logCurrentPos("Init");

            double expected_x = -27.5 - forwardOffset + armOrder[0];
            double expected_y =  stoneY * negativeMultiplier;
            drive.log("strafeTo(new Pose2d(" + expected_x + "," + expected_y);

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .strafeTo(new Vector2d(expected_x, expected_y))
                    .build());
            logCurrentPos("After initial move to stone");
            sleep(3000);
            logCurrentPos("Before final end");
//            if (checkForFailure(new Vector2d(-27 - forwardOffset + armOrder[0], stoneY * negativeMultiplier), startingPos.getHeading(), 2, 2, 10)) {
////                stopProgram();
//                logCurrentPos("Failed");
//            }
//
//            drive.setLogTag("main");
//            strategist.moveSkystoneArms(getArmSide(armOrder[0]), ArmStage.GRAB);
//            logCurrentPos("Grabbed 1st stone");
//            moveAndDrop(60 + armOrder[0], getArmSide(armOrder[0]));
//            logCurrentPos("After dropping 1st stone");
//
////            shiftPoseY(1);
//            // go back for 2nd stone
//            foundationToGrab(-47 - forwardOffset + armOrder[1], getArmSide(armOrder[1]));
//            logCurrentPos("Grabbed 2nd stone");
//
////            shiftPoseY(2);
//            moveAndDrop(53 + armOrder[1], getArmSide(armOrder[1]));
//            logCurrentPos("After dropping 2nd stone");
//
////            shiftPoseY(3);
//            foundationToGrab(-26 - thirdStoneOffset + armOrder[2], getArmSide(armOrder[2]));
//            logCurrentPos("Grabbed 3rd stone");
//
//            moveAndDrop(47 + armOrder[2], getArmSide(armOrder[2]));
//
//            sleep(1000);
///*
//            // Move Foundation
//            drive.turnToRadians(Math.toRadians(-90 * negativeMultiplier), strategist.getHeading());
//            drive.followTrajectorySync(drive.trajectoryBuilder()
//                    .forward(6) // forward
//                    .addMarker(0.1, () -> { strategist.moveFoundationServos(1); return null; }) //put mover down while moving
//                    .setReversed(true)
//                    .splineTo(new Pose2d(40, 48 * negativeMultiplier, Math.toRadians(45 * negativeMultiplier))) // turn
//                    .setReversed(false)
//                    .build());
//            drive.turnToRadians(0, strategist.getHeading()); // make another turn
//
//            // Push to wall and Park
//            drive.followTrajectorySync(drive.trajectoryBuilder()
//                    .forward(54 - drive.getPoseEstimate().getX() + 4) // push foundation to wall
//                    .addMarker(0, () -> { strategist.moveFoundationServos(0); return null;}) // open mover
//                    .back(5)
//                    .setReversed(true)
//                    .splineTo(new Pose2d(0, 34 * negativeMultiplier, 0))
//                    .build());
//
// */
        }
    }

    private void setUpVariables() {
        if (teamColor == TeamColor.RED) {
            negativeMultiplier = -1;
            startingPos = new Pose2d(-32, -63, Math.toRadians(180));

            if (getSkystonePos() == SkystonePos.LEFT) thirdStoneOffset = redForwardOffsets[1];
            else thirdStoneOffset = redForwardOffsets[2];
            forwardOffset = strategist.getForwardOffset(getSkystonePos(), redForwardOffsets);

            if (getSkystonePos() == SkystonePos.LEFT)        armOrder = new double[] {backArm, backArm, frontArm};
            else if (getSkystonePos() == SkystonePos.MIDDLE) armOrder = new double[] {backArm, frontArm, frontArm};
            else if (getSkystonePos() == SkystonePos.RIGHT)  armOrder = new double[] {frontArm, frontArm, frontArm};

        }
        else if (teamColor == TeamColor.BLUE) {
            negativeMultiplier = 1;
            startingPos = new Pose2d(-32, 61.5, 0);

            if (getSkystonePos() == SkystonePos.LEFT) thirdStoneOffset = blueForwardOffsets[1];
            else thirdStoneOffset = blueForwardOffsets[0];
            forwardOffset = strategist.getForwardOffset(getSkystonePos(), blueForwardOffsets);

            if (getSkystonePos() == SkystonePos.LEFT)        armOrder = new double[] {frontArm, frontArm, backArm};
            else if (getSkystonePos() == SkystonePos.MIDDLE) armOrder = new double[] {frontArm, backArm, backArm};
            else if (getSkystonePos() == SkystonePos.RIGHT)  armOrder = new double[] {backArm, backArm, backArm};
        }
        drive.setPoseEstimate(startingPos);

        buildingZone = new Pose2d(buildingZone.getX(), buildingZone.getY() * negativeMultiplier, buildingZone.getHeading());
        loadingZone = new Pose2d(loadingZone.getX(), loadingZone.getY() * negativeMultiplier, loadingZone.getHeading());
        centerLine = new Pose2d(centerLine.getX(), centerLine.getY() * negativeMultiplier, centerLine.getHeading());
    }

    private void moveAndDrop(double foundationX, ArmSide arm) {
        drive.setLogTag("moveAndDrop");
//        logCurrentPos("before trajectory");
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeLeft(2)
//                .splineTo(loadingZone)
//                .splineTo(buildingZone)
                .splineTo(centerLine)
                .splineTo(new Pose2d(foundationX, (foundationY + 3) * negativeMultiplier, 0))
                .strafeTo(new Vector2d(foundationX, foundationY * negativeMultiplier))
                .build());
        strategist.moveSkystoneArms(arm, ArmStage.DROP);
//        logCurrentPos("after trajectory");
    }

    private void foundationToGrab(double stoneX, ArmSide arm) {
        drive.setLogTag("foundationToGrab");
//        logCurrentPos("after trajectory");

        drive.log("strafeLeft(2)");
        drive.log("addMarker(0.3, strategist.resetSkystoneArms()");
        drive.log("splineTo(centerLine)");
        drive.log("splineTo(new Pose2d(" + stoneX + "," + (stoneY + 5) * negativeMultiplier + ", 0)");
        drive.log("strafeTo(new Pose2d(" + stoneX + "," + stoneY * negativeMultiplier);
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeLeft(2) // move away from foundation
                .addMarker(0.3, () -> { strategist.resetSkystoneArms(); return null;})
                .setReversed(true)
//                .splineTo(buildingZone)
//                .splineTo(loadingZone)
                .splineTo(centerLine)
                .addMarker(loadingZone.vec(), () -> { strategist.moveSkystoneArms(arm, ArmStage.OPENGRABBER); strategist.moveSkystoneArms(arm, ArmStage.PREPAREARM); return null;})
                .splineTo(new Pose2d(stoneX, (stoneY + 5) * negativeMultiplier, 0))
                .setReversed(false)
                .strafeTo(new Vector2d(stoneX, stoneY * negativeMultiplier))
                .build());
        strategist.moveSkystoneArms(arm, ArmStage.GRAB);
//        logCurrentPos("after trajectory");
    }

    private void logCurrentPos(String context) {
        drive.log(context + " current Position:" + drive.getPoseEstimate().getX() + "," + drive.getPoseEstimate().getY());
    }

    private ArmSide getArmSide(double side) {
        if (side == frontArm) return ArmSide.FRONT;
        else if (side == backArm) return ArmSide.BACK;
        return ArmSide.BACK;
    }

    private void shiftPoseY(double shift) {
        drive.setPoseEstimate(drive.getPoseEstimate().minus(new Pose2d(0, shift * negativeMultiplier, 0)));
    }

    private void setPoseYTo(double Y) {
        drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), Y, drive.getPoseEstimate().getHeading()));
    }

    private boolean checkForFailure(Vector2d targetPos, double targetHeading, double toleranceX, double toleranceY, double toleranceHeadingDeg) {
        drive.update();
        Pose2d currentPos = drive.getPoseEstimate();
        boolean failedX = false, failedY = false, failedHeading = false;
        if (Math.abs(currentPos.getX()       - targetPos.getX() ) > toleranceX)      failedX = true;
        if (Math.abs(currentPos.getY()       - targetPos.getY() ) > toleranceY)      failedY = true;
        if (Math.abs(currentPos.getHeading() - targetHeading    ) > Math.toRadians(toleranceHeadingDeg)) failedHeading = true;
        drive.setLogTag("Fail Check");
        drive.log("Current Pos:  " + currentPos.toString());
        drive.log("Supposed Pos: " + new Pose2d(targetPos, targetHeading).toString());
        drive.log("FailedX: " + failedX);
        drive.log("FailedY: " + failedY);
        drive.log("FailedHeading: " + failedHeading);

        return failedX || failedY || failedHeading;
    }
    private void stopProgram() {
        drive.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
        drive.setMotorPowers(0, 0, 0, 0); // Just in case still moving
        sleep(2000);
        requestOpModeStop();
    }
}

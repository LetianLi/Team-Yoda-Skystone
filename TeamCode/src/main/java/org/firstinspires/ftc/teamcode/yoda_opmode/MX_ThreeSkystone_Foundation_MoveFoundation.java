package org.firstinspires.ftc.teamcode.yoda_opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.yoda_code.AutonomousBase;
import org.firstinspires.ftc.teamcode.yoda_enum.ArmSide;
import org.firstinspires.ftc.teamcode.yoda_enum.ArmStage;
import org.firstinspires.ftc.teamcode.yoda_enum.SkystonePos;
import org.firstinspires.ftc.teamcode.yoda_enum.TeamColor;

@Autonomous(group = "auto", name = "MX: 3 Skystone->Foundation->Move it->park ")
public class MX_ThreeSkystone_Foundation_MoveFoundation extends AutonomousBase {

    private double[] blueForwardOffsets = {0, 8, 16}; // left, middle, right
    private double[] redForwardOffsets  = {16, 8, 0}; // left middle, right
    private double forwardOffset = 0;
    private double neg = 1;

    private double stoneY = 33;
    private double foundationY = 0;

    private double[] armOrder;
    private double frontArm = -5;
    private double backArm = 7;

    private double thirdStoneOffset = blueForwardOffsets[0];

    private Pose2d buildingZone = new Pose2d(20, 41, 0);
    private Pose2d loadingZone = new Pose2d(-12, 41, 0);
    private Pose2d centerLineStone = new Pose2d(0, 41, 0);
    private Pose2d centerLinePlain = new Pose2d(0, 38, 0);

    private Pose2d startingPos;
    private double baseHeading = 0;
    private boolean reversed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        if (isStopRequested()) return;
        if (strategist != null) {

            setUpVariables();

            drive.setLogTag("main");
            logCurrentPos("*** Plan to move to pick up 1st stone");
            drive.resetLatencyTimer();

            double expected_x = -27 - forwardOffset + armOrder[0] * neg;
            double expected_y =  stoneY * neg;
            drive.log("splineTo(new Pose2d(" + expected_x + "," + expected_y);
            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .addMarker(0, () -> {
                        strategist.moveSkystoneArms(getArmSide(armOrder[0]), ArmStage.LOWERARM);
                        strategist.moveSkystoneArms(getArmSide(armOrder[0]), ArmStage.OPENGRABBER);
                        return null;})
                    .strafeTo(new Vector2d(expected_x, expected_y))
                    .addMarker(new Vector2d(expected_x, expected_y + 4 * neg), () -> { strategist.moveSkystoneArms(getArmSide(armOrder[0]), ArmStage.CLOSEGRABBER); return null;})
                    .build());
            //if (checkForFailure(new Vector2d(expected_x, expected_y), startingPos.getHeading(), 2, 2, 10)) {
//                stopProgram();
                //logCurrentPos("Failed");
            //}
            logError("*** stone 1 | after moving to stone", expected_x, expected_y);

            drive.setLogTag("main");
            strategist.moveSkystoneArms(getArmSide(armOrder[0]), ArmStage.GRAB);
            logCurrentPos("*** Grabbed 1st stone");

            double y_error = moveAndDrop(
                    1,
                    59 + armOrder[0] * neg,
                    getArmSide(armOrder[0]),
                    teamColor == TeamColor.BLUE ? -1: -3,
                    teamColor == TeamColor.BLUE ? 1 : 1);
            logCurrentPos("After dropping 1st stone");
            double offset = 0;
            if (y_error > 2.0) {
                offset = y_error - 2;
            }

            // go back for 2nd stone
            foundationToGrab(2, -49 - forwardOffset + armOrder[1] * neg, getArmSide(armOrder[1]), teamColor == TeamColor.BLUE ? offset : 0, 0);
            logCurrentPos("Grabbed 2nd stone");

            y_error = moveAndDrop(
                    2,
                    50 + armOrder[1] * neg,
                    getArmSide(armOrder[1]),
                    teamColor == TeamColor.BLUE ? 1 : -1,
                    0);
            logCurrentPos("After dropping 2nd stone");

            offset = y_error;

            // go back for 3rd stone
            foundationToGrab(3, -26 - thirdStoneOffset + armOrder[2] * neg, getArmSide(armOrder[2]), teamColor == TeamColor.BLUE ? offset : 0, teamColor == TeamColor.BLUE ? 2 : 2);
            logCurrentPos("Grabbed 3rd stone");

            moveAndDrop(
                    3,
                    39 + armOrder[2] * neg,
                    getArmSide(armOrder[2]),
                    teamColor == TeamColor.BLUE ? offset : 0,
                    2);
            logCurrentPos("After dropping 3rd stone");

            // Move Foundation
            drive.strafeLeft(3);
            drive.turnToRadians(Math.toRadians(-90 * neg), drive.getHeading());

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .addMarker(0, () -> {
                        strategist.resetSkystoneArms(); // get arm back to position.
                        drive.parkingArm.setPosition(1);
                        strategist.moveFoundationServos(0.7);
                        return null;})
                    //.strafeTo(new Vector2d(50, drive.getY() + 1))
                    .strafeTo(new Vector2d(50, drive.getY() - 5 * neg)) // forward
                    .addMarker(new Vector2d(50, drive.getY() - 4.5 * neg), () -> { strategist.moveFoundationServos(1); return null; }) //put mover down while moving
                    .build());
            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .setReversed(true)
                    .splineTo(new Pose2d(35, (teamColor == TeamColor.BLUE ? 50 : 40) * neg, Math.toRadians(20 * neg))) // turn
                    .build());
//            drive.turnToRadians(0, drive.getHeading()); // make another turn

            // Push to wall and Park
            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .forward(10) // push foundation to wall
                    .addMarker(0.1, () -> { strategist.moveFoundationServos(0.7); return null;}) // open mover
                    .build());

//            if (teamColor == TeamColor.BLUE) setPoseYToDistance();

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    //.strafeTo(new Vector2d(drive.getX() - 5, drive.getY() - 5 * neg))
                    .strafeTo(new Vector2d(0, 38 * neg))
                    .build());

        }
    }

    private void setUpVariables() {
        if (teamColor == TeamColor.RED) {
            neg = -1;
            baseHeading = Math.toRadians(180);
            reversed = true;
            startingPos = new Pose2d(-39.5, -61.5, baseHeading);

            if (getSkystonePos() == SkystonePos.RIGHT) thirdStoneOffset = redForwardOffsets[1];
            else thirdStoneOffset = redForwardOffsets[2];
            forwardOffset = strategist.getForwardOffset(getSkystonePos(), redForwardOffsets);

            if (getSkystonePos() == SkystonePos.LEFT)        armOrder = new double[] {backArm, frontArm, frontArm};
            else if (getSkystonePos() == SkystonePos.MIDDLE) armOrder = new double[] {backArm, frontArm, frontArm};
            else if (getSkystonePos() == SkystonePos.RIGHT)  armOrder = new double[] {backArm, frontArm, frontArm};
            foundationY = 29;
        }
        else if (teamColor == TeamColor.BLUE) {
            neg = 1;
            baseHeading = 0;
            reversed = false;
            startingPos = new Pose2d(-32, 61.5, baseHeading);

            if (getSkystonePos() == SkystonePos.LEFT) thirdStoneOffset = blueForwardOffsets[1];
            else thirdStoneOffset = blueForwardOffsets[0];
            forwardOffset = strategist.getForwardOffset(getSkystonePos(), blueForwardOffsets);

            if (getSkystonePos() == SkystonePos.LEFT)        armOrder = new double[] {frontArm, frontArm, backArm};
            else if (getSkystonePos() == SkystonePos.MIDDLE) armOrder = new double[] {frontArm, backArm, backArm};
            else if (getSkystonePos() == SkystonePos.RIGHT)  armOrder = new double[] {frontArm, backArm, backArm};
            foundationY = 34;
        }
        drive.setPoseEstimate(startingPos);
        drive.resetLeftSensorToWall(startingPos.getY(), 0);

        buildingZone = new Pose2d(buildingZone.getX(), buildingZone.getY() * neg, baseHeading);
        loadingZone = new Pose2d(loadingZone.getX(), loadingZone.getY() * neg, baseHeading);
        centerLineStone = new Pose2d(centerLineStone.getX(), centerLineStone.getY() * neg, baseHeading);
        centerLinePlain = new Pose2d(centerLinePlain.getX(), centerLinePlain.getY() * neg, baseHeading);

        drive.turnLedOff();
    }

    private double moveAndDrop(int stoneIndex, double foundationX, ArmSide arm, double center_y_offset, double foundation_y_offset) {
        auto_timer.reset();
        drive.setLogTag("moveAndDrop for stone " + stoneIndex);
//        logCurrentPos("before trajectory");
        double expected_x = foundationX;
        double expected_y = (foundationY + foundation_y_offset) * neg;
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeTo(new Vector2d(drive.getX(), drive.getY() + 3 * neg))
                .setReversed(reversed)
//                .splineTo(loadingZone)
//                .splineTo(buildingZone)
                .splineTo(centerLineStone.plus(new Pose2d(0, center_y_offset * neg)))
                .addMarker(buildingZone.vec(), () -> { strategist.moveSkystoneArms(arm, ArmStage.PREPDROP); return null;})
                .splineTo(new Pose2d(foundationX, (foundationY + 5 + foundation_y_offset) * neg, baseHeading))
                .strafeTo(new Vector2d(expected_x, expected_y))
                .build());
        double y_error = logError("after moving to foundation", expected_x, expected_y);
        double rightDistance = drive.getRightDistance();
        double leftDistance = drive.getLeftDistance();
        drive.log("current distance right to foundation: " + rightDistance);
        drive.log("current distance left to wall: " + leftDistance);
        //drive.log("Calculated Y: " + drive.getCalculatedY(startingPos.getY()));
        strategist.moveSkystoneArms(arm, ArmStage.DROP);
        drive.log("Cycle time:" + auto_timer.milliseconds());
        return y_error;
//        logCurrentPos("after trajectory");
    }

    private void foundationToGrab(int stoneIndex, double stoneX, ArmSide arm, double center_y_offset, double stone_y_offset) {
        auto_timer.reset();
        drive.setLogTag("foundationToGrab for stone " + stoneIndex);
//        logCurrentPos("after trajectory");
        double expected_x = stoneX;
        double expected_y = (stoneY + stone_y_offset) * neg;

        drive.log("strafeTo(new Vector2d( " + drive.getX() + ", " + (drive.getY() + 2 * neg) + ")");
        drive.log("addMarker(0.3, strategist.resetSkystoneArms()");
        drive.log("splineTo(centerLinePlain)");
        drive.log("splineTo(new Pose2d(" + stoneX + "," + (stoneY + 5 + center_y_offset) * neg + ", " + baseHeading + ")");
        drive.log("strafeTo(new Pose2d(" + stoneX + "," + stoneY * neg);
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeTo(new Vector2d(drive.getX(), drive.getY() + 2 * neg)) // move away from foundation
                .addMarker(0.3, () -> {
                    strategist.resetSkystoneArms();
                    return null;})
                .setReversed(!reversed)
                .splineTo(centerLinePlain.plus(new Pose2d(0, center_y_offset * neg)))
                .addMarker(centerLinePlain.vec().plus(new Vector2d(getArmOffset(arm) * neg - 5, center_y_offset * neg)), () -> {
                    strategist.moveSkystoneArms(arm, ArmStage.OPENGRABBER);
                    return null;})
                .addMarker(centerLinePlain.vec().plus(new Vector2d(getArmOffset(arm) * neg - 15, center_y_offset * neg)), () -> {
                    strategist.moveSkystoneArms(arm, ArmStage.PREPARM);
                    return null;})
                .splineTo(new Pose2d(stoneX, (stoneY  + stone_y_offset + 6) * neg, baseHeading))
                .addMarker(new Vector2d(stoneX, (stoneY  + stone_y_offset + 3) * neg), () -> {
                    strategist.moveSkystoneArms(arm, ArmStage.LOWERARM);
                    return null;})
                .strafeTo(new Vector2d(expected_x, expected_y))
                .build());
        logError("after moving from foundation to stones", expected_x, expected_y);
        double currentDistance = drive.getRightDistance();
        drive.log("current distance right:" + currentDistance);
        if (currentDistance > 2.5 && currentDistance < 8) {
            drive.log("strafeRight:"+ (currentDistance - 2.5));
            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .strafeRight(currentDistance - 2.5)
                    .addMarker(new Vector2d(expected_x, expected_y + (currentDistance + 1) * neg), () -> {
                        strategist.moveSkystoneArms(arm, ArmStage.CLOSEGRABBER);
                        return null;})
                    .build());
            sleep(300);
            logError("after strafeRight", expected_x, expected_y - (currentDistance - 2.5));
        }
        else {
            strategist.moveSkystoneArms(arm, ArmStage.CLOSEGRABBER);
            sleep(300);
        }
        strategist.moveSkystoneArms(arm, ArmStage.GRAB);
        drive.log("Cycle time:" + auto_timer.milliseconds());
    }

    private void logCurrentPos(String context) {
        drive.update();
        Pose2d currentPos = drive.getPoseEstimate();

        drive.log(context + " current Position:" + currentPos.getX() + "," + currentPos.getY()
                + ", heading:" + Math.toDegrees(currentPos.getHeading()));
                //+ "\n imu:" + Math.toDegrees(drive.getRawExternalHeading()));
        ;
    }

    private ArmSide getArmSide(double side) {
        if (side == frontArm) return ArmSide.FRONT;
        else if (side == backArm) return ArmSide.BACK;
        return ArmSide.FRONT;
    }

    private double getArmOffset(ArmSide armSide) {
        switch (armSide){
            case FRONT: return frontArm;
            case BACK: return backArm;
            default: return frontArm;
        }
    }

//    private void shiftPoseY(double shift) {
//        drive.setPoseEstimate(drive.getPoseEstimate().minus(new Pose2d(0, shift * neg, 0)));
//    }

    private void setPoseYToDistance() {
        Pose2d newPose = new Pose2d(drive.getX(), drive.getCalculatedY(startingPos.getY()), drive.getHeading());
        drive.log("Current Position: " + drive.getPoseEstimate().toString());
        drive.log("New Position: " + newPose.toString());
        drive.setPoseEstimate(newPose);
    }

/*
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
    }*/
}

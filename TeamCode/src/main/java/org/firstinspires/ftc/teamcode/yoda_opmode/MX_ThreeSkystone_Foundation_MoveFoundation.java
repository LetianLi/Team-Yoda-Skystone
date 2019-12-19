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
    private double negativeMultiplier = 1;

    private double innerPositionOffset = 30;
    private double middlePositionOffset = 22;
    private double outterPositionOffset = 14;
    private double stoneY = 32;
    private double foundationY = 33;

    private double[] armOrder;
    private double frontArm = -5;
    private double backArm = 7;

    private double thirdStoneOffset = blueForwardOffsets[0];

    private Pose2d buildingZone = new Pose2d(20, 39, 0);
    private Pose2d loadingZone = new Pose2d(-12, 39, 0);
    private Pose2d centerLine = new Pose2d(0, 39, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        if (isStopRequested()) return;
        if (strategist != null) {

            setUpVariables();
            strategist.moveSkystoneArms(getArmSide(armOrder[0]), ArmStage.PREPARE);
            strategist.moveSkystoneArms(getArmSide(armOrder[0]), ArmStage.OPENGRABBER);

            drive.setLogTag("main");
            logCurrentPos("Init");

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .strafeTo(new Vector2d(-27 - forwardOffset + armOrder[0], stoneY * negativeMultiplier))
                    .build());
            strategist.moveSkystoneArms(getArmSide(armOrder[0]), ArmStage.GRAB);
            logCurrentPos("Grabbed 1st stone");
            moveAndDrop(29 + innerPositionOffset + armOrder[0], getArmSide(armOrder[0]));
            logCurrentPos("After dropping 1st stone");

            // go back for 2nd stone
            foundationToGrab(-51 - forwardOffset + armOrder[1], getArmSide(armOrder[1]));
            logCurrentPos("Grabbed 2nd stone");

            moveAndDrop(33 + middlePositionOffset + armOrder[1], getArmSide(armOrder[1]));
            logCurrentPos("After dropping 2nd stone");

            foundationToGrab(-27 - thirdStoneOffset + armOrder[2], getArmSide(armOrder[2]));
            logCurrentPos("Grabbed 3rd stone");
/*
            moveAndDrop(33 + outterPositionOffset + armOrder[2], getArmSide(armOrder[2]));

            // Move Foundation
            drive.turnToRadians(Math.toRadians(-90 * negativeMultiplier), strategist.getHeading());
            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .forward(6) // forward
                    .addMarker(0.1, () -> { strategist.moveFoundationServos(1); return null; }) //put mover down while moving
                    .setReversed(true)
                    .splineTo(new Pose2d(40, 48 * negativeMultiplier, Math.toRadians(45 * negativeMultiplier))) // turn
                    .setReversed(false)
                    .build());
            drive.turnToRadians(0, strategist.getHeading()); // make another turn

            // Push to wall and Park
            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .forward(54 - drive.getPoseEstimate().getX() + 4) // push foundation to wall
                    .addMarker(0, () -> { strategist.moveFoundationServos(0); return null;}) // open mover
                    .back(5)
                    .setReversed(true)
                    .splineTo(new Pose2d(0, 34 * negativeMultiplier, 0))
                    .build());

 */
        }
    }

    private void setUpVariables() {
        if (teamColor == TeamColor.RED) {
            negativeMultiplier = -1;
            drive.setPoseEstimate(new Pose2d(-32, -63, Math.toRadians(180)));

            if (getSkystonePos() == SkystonePos.LEFT) thirdStoneOffset = redForwardOffsets[1];
            else thirdStoneOffset = redForwardOffsets[2];
            forwardOffset = strategist.getForwardOffset(getSkystonePos(), redForwardOffsets);

            if (getSkystonePos() == SkystonePos.LEFT)        armOrder = new double[] {backArm, backArm, frontArm};
            else if (getSkystonePos() == SkystonePos.MIDDLE) armOrder = new double[] {backArm, frontArm, frontArm};
            else if (getSkystonePos() == SkystonePos.RIGHT)  armOrder = new double[] {frontArm, frontArm, frontArm};

        }
        else if (teamColor == TeamColor.BLUE) {
            negativeMultiplier = 1;
            drive.setPoseEstimate(new Pose2d(-32, 61.5, 0));

            if (getSkystonePos() == SkystonePos.LEFT) thirdStoneOffset = blueForwardOffsets[1];
            else thirdStoneOffset = blueForwardOffsets[0];
            forwardOffset = strategist.getForwardOffset(getSkystonePos(), blueForwardOffsets);

            if (getSkystonePos() == SkystonePos.LEFT)        armOrder = new double[] {frontArm, frontArm, backArm};
            else if (getSkystonePos() == SkystonePos.MIDDLE) armOrder = new double[] {frontArm, backArm, backArm};
            else if (getSkystonePos() == SkystonePos.RIGHT)  armOrder = new double[] {backArm, backArm, backArm};
        }

        buildingZone = new Pose2d(buildingZone.getX(), buildingZone.getY() * negativeMultiplier, buildingZone.getHeading());
        loadingZone = new Pose2d(loadingZone.getX(), loadingZone.getY() * negativeMultiplier, loadingZone.getHeading());
        centerLine = new Pose2d(centerLine.getX(), centerLine.getY() * negativeMultiplier, centerLine.getHeading());
    }

    private void moveAndDrop(double foundationX, ArmSide arm) {
        drive.setLogTag("moveAndDrop");
        logCurrentPos("before trajectory");
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeTo(new Vector2d(drive.getPoseEstimate().getX(), 35 * negativeMultiplier))
//                .splineTo(loadingZone)
//                .splineTo(buildingZone)
                .splineTo(centerLine)
                .splineTo(new Pose2d(foundationX, (foundationY + 3) * negativeMultiplier, 0))
                .strafeTo(new Vector2d(foundationX, foundationY * negativeMultiplier))
                .build());
        strategist.moveSkystoneArms(arm, ArmStage.DROP);
        logCurrentPos("after trajectory");
    }

    private void foundationToGrab(double stoneX, ArmSide arm) {
        drive.setLogTag("foundationToGrab");
        logCurrentPos("after trajectory");

        drive.log("strafeLeft(2)");
        drive.log("addMarker(strategist.resetSkystoneArms())");
        drive.log("splineTo(centerLine)");
        drive.log("splineTo(new Pose2d(" + stoneX + "," + (stoneY + 5) * negativeMultiplier);
        drive.log("strafeTo(new Pose2d(" + stoneX + "," + stoneY * negativeMultiplier);
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeLeft(2) // move away from foundation
                .addMarker(0.3, () -> { strategist.resetSkystoneArms(); return null;})
                .setReversed(true)
//                .splineTo(buildingZone)
//                .splineTo(loadingZone)
                .splineTo(centerLine)
                .addMarker(loadingZone.vec(), () -> { strategist.moveSkystoneArms(arm, ArmStage.OPENGRABBER); strategist.moveSkystoneArms(arm, ArmStage.PREPARE); return null;})
                .splineTo(new Pose2d(stoneX, (stoneY + 5) * negativeMultiplier, 0))
                .strafeTo(new Vector2d(stoneX, stoneY * negativeMultiplier))
                .setReversed(false)
                .build());
        strategist.moveSkystoneArms(arm, ArmStage.GRAB);
        logCurrentPos("after trajectory");
    }

    private void logCurrentPos(String context)
    {
        drive.log(context + " current Position:" + drive.getPoseEstimate().getX() + "," + drive.getPoseEstimate().getY());
    }

    private ArmSide getArmSide(double side) {
        if (side == frontArm) return ArmSide.FRONT;
        else if (side == backArm) return ArmSide.BACK;
        return ArmSide.BACK;
    }
}

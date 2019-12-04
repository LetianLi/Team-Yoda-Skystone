package org.firstinspires.ftc.teamcode.yoda_code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yoda_enum.ArmSide;
import org.firstinspires.ftc.teamcode.yoda_enum.ArmStage;

import static java.lang.Thread.sleep;

public class RedStrategist extends StrategistBase {
    private double forwardOffset = 0;
    private double[] forwardOffsetsPerPos = {3, 9, 19}; // left, middle, right
    private static double RIGHT_TO_STONE = 30;

    public RedStrategist(
            YodaMecanumDrive drive, ElapsedTime op_timer, AutonomousBase opMode) {
        super(drive, op_timer, opMode);
    }

    @Override
    public void calculateDistance() {

    }

    @Override
    public void grabSkyStone() {
        drive.setPoseEstimate(new Pose2d(-33, -63, Math.toRadians(180)));
        moveSkystoneArms(ArmSide.FRONT, ArmStage.PREPARE);

        this.forwardOffset = getForwardOffset(opMode.getSkystonePos(), forwardOffsetsPerPos);

        drive.strafeRight(RIGHT_TO_STONE);
        drive.turnTo(Math.toRadians(180));

        if (forwardOffset > 0) {
            drive.back(Math.abs(forwardOffset));
        }
        else if (forwardOffset < 0){
            drive.forward(Math.abs(forwardOffset));
        }

        moveSkystoneArms(ArmSide.FRONT, ArmStage.GRAB);
    }

    @Override
    public void moveAndDropSkystoneOnFoundation() {
        moveAndDropSkystoneOnFoundationWithForwardDistance(0);
    }

    @Override
    public void moveAndDropSkystoneOnFoundationWithForwardDistance(double extraForwards) {
        drive.strafeLeft(7);
        drive.turnTo(Math.toRadians(180));
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .back(79.5 - forwardOffset + extraForwards)
                .strafeRight(12.7)
                .build());
        moveSkystoneArms(ArmSide.FRONT, ArmStage.DROP);
    }

    @Override
    public void goBackGrabDeliverSecondSkystone() {
        drive.strafeLeft(7);
        drive.turnTo(Math.toRadians(180));
        drive.forward(77);
        double extraMoveForwardsDistance = moveForwardToDistance(11 + forwardOffset, false, 30);
        moveSkystoneArms(ArmSide.FRONT, ArmStage.PREPARE);
        drive.turnTo(Math.toRadians(180));
        moveRightToDistance(2, true, 5);
        moveSkystoneArms(ArmSide.FRONT, ArmStage.GRAB);
        moveAndDropSkystoneOnFoundationWithForwardDistance(extraMoveForwardsDistance + forwardOffset - 2);
    }


    @Override
    public void fromFoundationToPark() {
        drive.strafeLeft(7);
        drive.turnTo(Math.toRadians(180));
        drive.forward(55);
        drive.strafeRight(4);
    }

    private double moveForwardToDistance(double distanceFromFront, boolean doBackwardInCase, double maxMovementForward) {
        distanceFromFront -= 3;
        double currentDistance = drive.getFrontDistance() - 3;
        opMode.telemetry.addData("current distance front", currentDistance);
        opMode.telemetry.update();
        if (currentDistance > distanceFromFront && currentDistance - distanceFromFront <= maxMovementForward) {
            drive.forward(currentDistance - distanceFromFront);
        }
        else if (currentDistance - distanceFromFront >= maxMovementForward) {
            drive.forward(maxMovementForward);
            return maxMovementForward;
        }
        else if (distanceFromFront > currentDistance && doBackwardInCase) {
            drive.back(distanceFromFront - currentDistance);
        }
        return currentDistance - distanceFromFront;
    }

    @Override
    // Used in M5 only
    public void turnAndMoveFoundationAndPark() {
        drive.strafeLeft(6);
        drive.back(7);
        drive.turnTo(Math.toRadians(90));
        drive.forward(5);
        moveFoundationServos(1);
        drive.forward(2);
        updatePose();
        opMode.sleep(1000);
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .back(10)
                .setReversed(true)
                .splineTo(new Pose2d(getX() - 25, getY() - 15, 0))
                .build());
        drive.turnTo(0);
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .setReversed(false)
                .forward(20)
                .build());
        drive.turnTo(0);
        moveFoundationServos(0);
        updatePose();
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .setReversed(true)
                .splineTo(new Pose2d(getX() - 43, getY() + 13.5, 0))
                .strafeRight(1)
                .setReversed(false)
                .build());
    }

    @Override
    public void moveFoundationBackAndPark() {
        // adjust position
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeLeft(3)
                .back(3)
                .build());
        // turn to face foundation and move
        drive.turnTo(Math.toRadians(90));

        updatePose();
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(5)
                .addMarker(0, () -> { moveFoundationServos(1); return null; })
                .setReversed(true)
                .splineTo(new Pose2d(getX() + 20, getY() + 30, Math.toRadians(130)))
                .setReversed(false)
                .build());

        drive.turnTo(Math.toRadians(-10));
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(5)
                .addMarker(0, () -> { moveFoundationServos(0); return null;})
                .back(5)
                .strafeLeft(13)
                .back(50)
//                .addMarker(2  , () -> {readyForManual(); return null;})
                .build());

    }
}

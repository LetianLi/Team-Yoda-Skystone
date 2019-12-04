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

        strafeRight(RIGHT_TO_STONE);
        turnTo(Math.toRadians(180));

        if (forwardOffset > 0) {
            back(Math.abs(forwardOffset));
        }
        else if (forwardOffset < 0){
            forward(Math.abs(forwardOffset));
        }

        moveSkystoneArms(ArmSide.FRONT, ArmStage.GRAB);
    }

    @Override
    public void moveAndDropSkystoneOnFoundation() {
        moveAndDropSkystoneOnFoundationWithForwardDistance(0);
    }

    @Override
    public void moveAndDropSkystoneOnFoundationWithForwardDistance(double extraForwards) {
        strafeLeft(7);
        turnTo(Math.toRadians(180));
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .back(79.5 - forwardOffset + extraForwards)
                .strafeRight(12.7)
                .build());
        moveSkystoneArms(ArmSide.FRONT, ArmStage.DROP);
    }

    @Override
    public void goBackGrabDeliverSecondSkystone() {
        strafeLeft(7);
        turnTo(Math.toRadians(180));
        forward(77);
        double extraMoveForwardsDistance = moveForwardToDistance(11 + forwardOffset, false, 30);
        moveSkystoneArms(ArmSide.FRONT, ArmStage.PREPARE);
        turnTo(Math.toRadians(180));
        moveRightToDistance(2, true, 5);
        moveSkystoneArms(ArmSide.FRONT, ArmStage.GRAB);
        moveAndDropSkystoneOnFoundationWithForwardDistance(extraMoveForwardsDistance + forwardOffset - 2);
    }


    @Override
    public void fromFoundationToPark() {
        strafeLeft(7);
        turnTo(Math.toRadians(180));
        forward(55);
        strafeRight(4);
    }

    @Override
    // Used in M5 only
    public void turnAndMoveFoundationAndPark() {
        strafeLeft(6);
        back(7);
        turnTo(Math.toRadians(90));
        forward(5);
        moveFoundationServos(1);
        forward(2);
        updatePose();
        opMode.sleep(1000);
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .back(10)
                .setReversed(true)
                .splineTo(new Pose2d(getX() - 25, getY() - 15, 0))
                .build());
        turnTo(0);
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .setReversed(false)
                .forward(20)
                .build());
        turnTo(0);
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
        turnTo(Math.toRadians(90));

        updatePose();
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(5)
                .addMarker(0, () -> { moveFoundationServos(1); return null; })
                .setReversed(true)
                .splineTo(new Pose2d(getX() + 20, getY() + 30, Math.toRadians(130)))
                .setReversed(false)
                .build());

        turnTo(Math.toRadians(-10));
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

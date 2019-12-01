package org.firstinspires.ftc.teamcode.yoda_code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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


    // 5.5 , -5, 18
    @Override
    public void grabSkyStone() {
        drive.setPoseEstimate(new Pose2d(33, 63, 0));
        moveSkystoneArms(ArmSide.FRONT, ArmStage.PREPARE);

        this.forwardOffset = getForwardOffset(opMode.getSkystonePos(), forwardOffsetsPerPos);

        strafeRight(RIGHT_TO_STONE);
        turnTo(0);

        if (forwardOffset > 0) {
            back(Math.abs(forwardOffset));
        }
        else if (forwardOffset < 0){
            forward(Math.abs(forwardOffset));
        }

        moveSkystoneArms(ArmSide.FRONT, ArmStage.GRAB);
        opMode.sleep(1000);
    }

    @Override
    public void moveAndDropSkystoneOnFoundation() {
        moveAndDropSkystoneOnFoundation(0);
    }

    @Override
    public void moveAndDropSkystoneOnFoundation(double extraForwards) {
        strafeLeft(5);
        turnTo(0);
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .back(79.5 - forwardOffset + extraForwards)
                .strafeRight(12.7)
                .build());
        moveSkystoneArms(ArmSide.FRONT, ArmStage.DROP);
    }

    @Override
    public void goBackGrabDeliverSecondSkystone() {
        strafeLeft(7);
        turnTo(0);
        forward(77);
        double extraMoveForwardsDistance = moveForwardToDistance(6);
        moveSkystoneArms(ArmSide.FRONT, ArmStage.PREPARE);
        if (forwardOffset > 0) {
            back(Math.abs(forwardOffset));
        }
        else {
            forward(Math.abs(forwardOffset));
        }
        turnTo(0);
        moveRightToDistance(2, 5);
        moveSkystoneArms(ArmSide.FRONT, ArmStage.GRAB);
        moveAndDropSkystoneOnFoundation(extraMoveForwardsDistance);
    }


    @Override
    public void fromFoundationToPark() {
        strafeLeft(7);
        turnTo(0);
        forward(55);
        strafeRight(4);
    }

    @Override
    // Used in M5 only
    public void turnAndMoveFoundationAndPark() {
        strafeLeft(6);
        back(7);
        turnTo(Math.toRadians(-90));
        forward(5);
        moveFoundationServos(1);
        forward(2);
        updatePose();
        opMode.sleep(1000);
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .back(10)
                .setReversed(true)
                .splineTo(new Pose2d(getX() + 25, getY() + 15, Math.toRadians(180)))
                .build());
        turnTo(Math.toRadians(160));
        turnTo(Math.toRadians(180));
        turnTo(Math.toRadians(180));
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .setReversed(false)
                .forward(20)
                .build());
        turnTo(Math.toRadians(180));
        moveFoundationServos(0);
        updatePose();
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .setReversed(true)
                .splineTo(new Pose2d(getX() + 43, getY() - 13.5, Math.toRadians(180)))
                .strafeRight(1)
                .setReversed(false)
                .build());
    }

    public void moveFoundationBackAndPark() {
        strafeLeft(6);
        forward(3);
        turnTo(Math.toRadians(-90));
        forward(5);
        moveFoundationServos(1);
        forward(2);
        strafeLeft(5);
        turnTo(Math.toRadians(-90));
        back(45);
        moveFoundationServos(0);

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeLeft(34)
                .forward(27)
                .strafeLeft(24)
                .build());
    }
}

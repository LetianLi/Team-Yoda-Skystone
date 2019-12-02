package org.firstinspires.ftc.teamcode.yoda_code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yoda_enum.ArmSide;
import org.firstinspires.ftc.teamcode.yoda_enum.ArmStage;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

public class BlueStrategist extends StrategistBase {
    private double forwardOffset = 0;
    private double[] forwardOffsetsPerPos = {10, 3, -4}; // left, middle, right
    private static double RIGHT_TO_STONE = 30;

    public BlueStrategist(
            YodaMecanumDrive drive, ElapsedTime op_timer, AutonomousBase opMode) {
        super(drive, op_timer, opMode);
    }


    // 5.5 , -5, 18
    @Override
    public void grabSkyStone() {
        drive.setPoseEstimate(new Pose2d(-33, 63, 0));
        moveSkystoneArms(ArmSide.BACK, ArmStage.PREPARE);

        this.forwardOffset = getForwardOffset(opMode.getSkystonePos(), forwardOffsetsPerPos);

        strafeRight(RIGHT_TO_STONE);
        turnTo(0);
        //drive.updatePoseEstimate();
        /*drive.followTrajectorySync(drive.trajectoryBuilder()
                .lineTo(new Vector2d(forwardOffset))
                .build());*/

        if (forwardOffset > 0) {
            forward(Math.abs(forwardOffset));
        }
        else if (forwardOffset < 0){
            back(Math.abs(forwardOffset));
        }
        moveSkystoneArms(ArmSide.BACK, ArmStage.GRAB);
        opMode.sleep(500);
    }

    @Override
    public void moveAndDropSkystoneOnFoundation() {
        moveAndDropSkystoneOnFoundation(0);
    }

    @Override
    public void moveAndDropSkystoneOnFoundation(double extraForwards) {
        strafeLeft(7);
        turnTo(0);
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(70 - forwardOffset + extraForwards)
                .strafeRight(15)
                .build());
        moveSkystoneArms(ArmSide.BACK, ArmStage.DROP);
    }

    @Override
    public void goBackGrabDeliverSecondSkystone() {
        strafeLeft(7);
        turnTo(0);
        back(90);
        double extraMoveBackDistance = moveBackToDistance(11 + forwardOffset, false, 50);
        moveSkystoneArms(ArmSide.BACK, ArmStage.PREPARE);
        turnTo(0);
        moveRightToDistance(2, true, 5);
        moveSkystoneArms(ArmSide.BACK, ArmStage.GRAB);
        moveAndDropSkystoneOnFoundation(extraMoveBackDistance + 0);
    }


    @Override
    public void fromFoundationToPark() {
        strafeLeft(5);
        turnTo(0);
        back(50);
        strafeRight(6);
    }

    @Override
    // Used in M5 only
    public void turnAndMoveFoundationAndPark() {
        strafeLeft(6);
        forward(3);
        turnTo(Math.toRadians(-90));
        forward(5);
        moveFoundationServos(1);
        forward(2);
        updatePose();
        opMode.sleep(500);
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .back(10)
                .setReversed(true)
                .splineTo(new Pose2d(getX() - 25, getY() + 15, 0))
                .build());
        turnTo(0);
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .setReversed(false)
                .forward(20)
                .build());
        moveFoundationServos(0);
        updatePose();
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .setReversed(true)
                .splineTo(new Pose2d(getX() - 43, getY() - 10, 0))
                .strafeRight(1)
                .setReversed(false)
                .build());
    }

    public void moveFoundationBackAndPark() {
        // adjust position
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeLeft(3)
                .forward(3)
                .build());
        // turn to face foundation and move
        turnTo(Math.toRadians(-90));
//        drive.followTrajectorySync(drive.trajectoryBuilder()
//                .forward(5)
//                .addMarker(0.5, () -> { moveFoundationServos(1); return null; })
//                .strafeLeft(6)
//                .build());
        updatePose();
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(5)
                .addMarker(0, () -> { moveFoundationServos(1); return null; })
                .setReversed(true)
                .splineTo(new Pose2d(getX() - 20, getY() + 30, Math.toRadians(-50)))
                .setReversed(false)
                .build());

        turnTo(Math.toRadians(10));
        turnTo(Math.toRadians(0));
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(5)
                .addMarker(0, () -> { moveFoundationServos(0); return null;})
                .back(5)
                .strafeRight(13)
                .back(50)
                .addMarker(2  , () -> {readyForManual(); return null;})
                .build());



        /*
        turnTo(Math.toRadians(-90));
        back(45);
        moveFoundationServos(0);

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(1)
                .back(1)
                .strafeRight(38)
                .forward(27)
                .strafeRight(28)
                .build());

         */
    }
}

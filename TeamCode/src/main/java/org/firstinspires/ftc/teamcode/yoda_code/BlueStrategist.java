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
        // tell road runner of our initial position, so that it can draw position in dashboard
        drive.setPoseEstimate(new Pose2d(-33, 63, 0));
        moveSkystoneArms(ArmSide.BACK, ArmStage.PREPARE);


        strafeRight(RIGHT_TO_STONE); // Move right to be close to stone
        turnTo(0); // adjust in case robot drift

        // Depends on position, move forward/backward to stone position
        this.forwardOffset = getForwardOffset(opMode.getSkystonePos(), forwardOffsetsPerPos);
        if (forwardOffset > 0) {
            forward(Math.abs(forwardOffset));
        }
        else if (forwardOffset < 0){
            back(Math.abs(forwardOffset));
        }
        moveSkystoneArms(ArmSide.BACK, ArmStage.GRAB);
    }

    @Override
    public void moveAndDropSkystoneOnFoundation() {
        moveAndDropSkystoneOnFoundation(0);
    }

    @Override
    public void moveAndDropSkystoneOnFoundation(double extraForwards) {
        strafeLeft(7); // move away from the stone, so that we do not hit bridge
        turnTo(0); // adjust in case robot drift
        // Move forward, then move right to be closer to foundation
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(70 - forwardOffset + extraForwards)
                .strafeRight(15)
                .build());
        moveSkystoneArms(ArmSide.BACK, ArmStage.DROP); // Drop the stone on foundation
    }

    @Override
    public void goBackGrabDeliverSecondSkystone() {
        strafeLeft(7); // move away from the foundation, so that we do not hit bridge
        turnTo(0);
        back(87); // Back, we may want to add offsite directly here to reduce one step
        // use sensor to read distance to back, and move to 2nd skystone positions
        double extraMoveBackDistance = moveBackToDistance(11 + forwardOffset, false, 30);
        moveSkystoneArms(ArmSide.BACK, ArmStage.PREPARE);
        turnTo(0);
        // move right until 2" close to the skystone
        moveRightToDistance(2, true, 5);
        moveSkystoneArms(ArmSide.BACK, ArmStage.GRAB);
        // move forward and drop stone to foundation again
        moveAndDropSkystoneOnFoundation(extraMoveBackDistance + forwardOffset - 2);
    }

    public void moveFoundationBackAndPark() {
        // adjust position
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeLeft(3)// away from foundation, avoid hitting it when turning
                .forward(3) // forward a little for positions
                .build());
        // turn to face foundation and move
        turnTo(Math.toRadians(-90));

        updatePose();
        // turn almost 90 degree to drag it
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(5) // forward
                .addMarker(0, () -> { moveFoundationServos(1); return null; }) //put mover down while moving
                .setReversed(true)
                .splineTo(new Pose2d(getX() - 20, getY() + 25, Math.toRadians(-50)))// turn
                .setReversed(false)
                .build());

        turnTo(Math.toRadians(0)); // make another turn
        // moving away to park
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(5) // push foundation to wall
                .addMarker(0, () -> { moveFoundationServos(0); return null;}) // open mover
                .back(5)// away from foundation, avoid hitting it
                .strafeRight(15)// right to position of parking
                .back(45) // back to parking position
                .build());
    }


    @Override
    // Used in M4 and M9
    public void fromFoundationToPark() {
        strafeLeft(7);
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

}

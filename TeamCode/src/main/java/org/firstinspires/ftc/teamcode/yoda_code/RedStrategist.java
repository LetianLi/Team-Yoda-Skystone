package org.firstinspires.ftc.teamcode.yoda_code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yoda_enum.ArmSide;
import org.firstinspires.ftc.teamcode.yoda_enum.ArmStage;

import static java.lang.Thread.sleep;

public class RedStrategist extends StrategistBase {
    private double forwardOffset = 0;
    private double[] forwardOffsetsPerPos = {11, 3, -5}; // left, middle, right
    private static double RIGHT_TO_STONE = 30;

    public RedStrategist(
            YodaMecanumDrive drive, ElapsedTime op_timer, AutonomousBase opMode) {
        super(drive, op_timer, opMode);
    }


    // 5.5 , -5, 18
    @Override
    public void GrabSkyStone() {
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
        else {
            back(Math.abs(forwardOffset));
        }
        moveSkystoneArms(ArmSide.BACK, ArmStage.GRAB);
        opMode.sleep(1000);
    }

    @Override
    public void moveAndDropSkystoneOnFoundation() {
        strafeLeft(5);
        turnTo(0);
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(80 - forwardOffset)
                .strafeRight(8)
                .build());
        moveSkystoneArms(ArmSide.BACK, ArmStage.DROP);
    }

    @Override
    public void fromFoundationToPark() {
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeLeft(7)
                .back(60)
                .build());
    }

    @Override
    public void DoubleSkystoneDelivery() {
    }

    @Override
    // Used in M5 only
    public void turnAndMoveFoundationAndPark() {
        strafeLeft(5);
        drive.turnSync(Math.toRadians(-90));
        forward(7);
        moveFoundationServos(1);
        updatePose();
        opMode.sleep(1000);
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .setReversed(true)
                .splineTo(new Pose2d(getX() - 20, getY() + 10, 0))
                .setReversed(false)
                .forward(25)
                .build());
        moveFoundationServos(0);
        updatePose();
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .setReversed(true)
                .splineTo(new Pose2d(getX() - 43, getY() - 3, 0))
                .setReversed(false)
                .build());
    }
}

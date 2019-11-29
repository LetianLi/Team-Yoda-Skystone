package org.firstinspires.ftc.teamcode.yoda_code;

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

        switch(opMode.getSkystonePos()) {
            case LEFT:
                forwardOffset = forwardOffsetsPerPos[0];
                break;
            case MIDDLE:
                forwardOffset = forwardOffsetsPerPos[1];
                break;
            case RIGHT:
                forwardOffset = forwardOffsetsPerPos[2];
                break;
            case UNKNOW:
                forwardOffset = forwardOffsetsPerPos[1];
                break;
        }

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
        strafeLeft(5);
    }

    @Override
    public void moveAndDropSkystoneOnFoundation() {

    }

    @Override
    public void fromFoundationToPark() {
    }

    @Override
    public void DoubleSkystoneDelivery() {
    }

    @Override
    // Used in M5 only
    public void turnAndMoveFoundationAndPark() {
    }
}

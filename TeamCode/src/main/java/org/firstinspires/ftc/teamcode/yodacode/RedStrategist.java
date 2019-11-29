package org.firstinspires.ftc.teamcode.yodacode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RedStrategist extends StrategistBase {
    private double forwardOffset = 0;
    private double[] forwardOffsetsPerPos = {1, 9, 19}; // right, middle, left
    private static double FIRST_RIGHT_DETECT_STONE = 21;
    private static double SECOND_RIGHT_CLOSER_TO_STONE = 18.5;

    public RedStrategist(
            YodaMecanumDrive drive, ElapsedTime op_timer, LinearOpMode opMode) {
        super(drive, op_timer, opMode);
    }

    @Override
    public void DetectAndGrabSkyStone() {
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

package org.firstinspires.ftc.teamcode.yodacode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class StrategistBase {
    protected YodaMecanumDrive drive;
    protected ElapsedTime op_timer;
    protected LinearOpMode opMode;

    public StrategistBase(
            YodaMecanumDrive drive,
            ElapsedTime op_timer,
            LinearOpMode opMode) {
        this.drive = drive;
        this.op_timer = op_timer;
        this.opMode = opMode;
    }

    public abstract void DetectAndGrabSkyStone();

    public abstract void moveAndDropSkystoneOnFoundation();

    public abstract void fromFoundationToPark();

    public abstract void DoubleSkystoneDelivery();

    public abstract void turnAndMoveFoundationAndPark();
}

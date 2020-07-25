package org.firstinspires.ftc.teamcode.yoda_code;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.List;

public class OdometryWheel {
    private ExpansionHubMotor wheel;
    private Pose2d offsetFromRobotCenter;
//    private List<TimePose> previousPositions;

    private final double WheelRadius;
    private final double GearRatio;
    private final double TicksPerRev;
    private boolean Reversed = false;

//    private final int VelocityTickLookahead = 5;

//    public class TimePose {
//        long time;
//        int ticks;
//        double inches;
//
//        public TimePose(int ticks) {
//            this.time = System.currentTimeMillis();
//            this.ticks = ticks;
//            this.inches = encoderTicksToInches(ticks);
//        }
//    }

    public OdometryWheel(HardwareMap hardwareMap, String deviceName, double wheelRadius, double gearRatio, double ticksPerRev, boolean reversed, Pose2d offsetFromRobotCenter) {
        this.wheel = hardwareMap.get(ExpansionHubMotor.class, deviceName);
        this.offsetFromRobotCenter = offsetFromRobotCenter;

        this.WheelRadius = wheelRadius;
        this.GearRatio = gearRatio;
        this.TicksPerRev = ticksPerRev;
        this.Reversed = reversed;

        if (reversed) setDirection(DcMotorSimple.Direction.REVERSE);
        else setDirection(DcMotorSimple.Direction.FORWARD);

        Log.i("Odometry Wheel", deviceName + " created with " + String.format("(WR: %.3f, GR: %.1f, TpR: %.0f, Reversed: ", WheelRadius, GearRatio, TicksPerRev) + Reversed + ")");
    }

    public void setMode(DcMotor.RunMode mode) {
        wheel.setMode(mode);
    }

    public void resetEncoder() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        wheel.setDirection(direction);
    }

    public Pose2d getWheelPosition() {
        return offsetFromRobotCenter;
    }

    /**
     * Gets position of odometry wheel to the robot
     */
    public int getCurrentPosition() {
        return wheel.getCurrentPosition();
    }

    public double getCurrentPositionInches() {
        return encoderTicksToInches(getCurrentPosition());
    }

    public double getVelocity() {
        return wheel.getVelocity();
    }

    public double getVelocityInches() {
//        if (previousPositions.size() < 2) {
//            return 0;
//        }
//
//        // We'd like to pick a time up to five reads ago, but we might not be able to
//        int oldIndex = Math.max(0, (previousPositions.size() - VelocityTickLookahead) - 1);
//        TimePose old = previousPositions.get(oldIndex);
//        TimePose current = previousPositions.get(previousPositions.size() - 1); // Finds end of list
//
//        double scaleFactor = (double) (current.time - old.time) / (1000); // Seconds elapsed
//        return (current.inches - old.inches) / scaleFactor;

        return encoderTicksToInches(getVelocity());
    }

    private double encoderTicksToInches(double ticks) {
        return WheelRadius * 2 * Math.PI * GearRatio * ticks / TicksPerRev;
    }
}

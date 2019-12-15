package org.firstinspires.ftc.teamcode.yoda_code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yoda_enum.ArmSide;
import org.firstinspires.ftc.teamcode.yoda_enum.ArmStage;
import org.firstinspires.ftc.teamcode.yoda_enum.SkystonePos;

import static java.lang.Thread.sleep;

public class RedStrategist extends StrategistBase {
    private double forwardOffset = 0;
    private double[] forwardOffsetsPerPos = {3, 10, 17}; // left, middle, right
    private static double RIGHT_TO_STONE = 30;
    private double firstBackwardDistance = 0;
    private double forwardDistanceFromFoundation = 0;
    private double secondBackwardDistance = 0;

    public RedStrategist(
            YodaMecanumDrive drive, ElapsedTime op_timer, AutonomousBase opMode) {
        super(drive, op_timer, opMode);
    }

    @Override
    public void calculateDistance() {
        drive.setLogTag("calculateDistance");
        // Depends on position, move forward/backward to stone position
        this.forwardOffset = getForwardOffset(opMode.getSkystonePos(), forwardOffsetsPerPos);
        drive.log("forwardOffset:" + forwardOffset);
        this.firstBackwardDistance = 90 - forwardOffset;
        drive.log("firstBackwardDistance:" + firstBackwardDistance);
        // Back, we move forward 80 - forwardOffset previously, now need to go back that much,
        // plus 3 stones distance 8*3, plus some extra for error accuracy
        this.forwardDistanceFromFoundation = firstBackwardDistance + 8 * 3 + 8;
        drive.log("forwardDistanceFromFoundation:" + forwardDistanceFromFoundation);
        // move forward and drop stone to foundation again, move 20" less so that we have room to
        // drop 2nd stone, +4 is for distance error when going backwards
        this.secondBackwardDistance = this.forwardDistanceFromFoundation - 20 + 4;
        drive.log("secondForwardDistance:" + secondBackwardDistance);
    }

    @Override
    public void grabSkyStone() {
        drive.setLogTag("grabSkyStone");
        drive.setPoseEstimate(new Pose2d(-33, -63, Math.toRadians(180)));
        moveSkystoneArms(ArmSide.FRONT, ArmStage.OPENGRABBER);

        drive.strafeRight(RIGHT_TO_STONE);
        drive.turnToRadians(0);

        if (forwardOffset > 0) {
            drive.back(Math.abs(forwardOffset));
        }
        else if (forwardOffset < 0){
            drive.forward(Math.abs(forwardOffset));
        }
        drive.log("Distance to stone:" + drive.getRightDistance());

        moveSkystoneArms(ArmSide.FRONT, ArmStage.GRAB);
    }

    @Override
    public void moveAndDropSkystoneOnFoundation() {
        moveAndDropSkystoneOnFoundationWithBackwardDistance(this.firstBackwardDistance);
    }

    private void moveAndDropSkystoneOnFoundationWithBackwardDistance(double backward_distance) {
        drive.setLogTag("moveAndDropSkystoneOnFoundationWithBackwardDistance");
        drive.strafeLeft(8); // move away from the stone, so that we do not hit bridge
        drive.turnToRadians(0); // adjust in case robot drift
        // Move forward, then move right to be closer to foundation
        double strafe_right_distance =  16;
        drive.log("followTrajectory: backward " + backward_distance + "+ strafeRight " + strafe_right_distance);
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .back(backward_distance)
                .strafeRight(strafe_right_distance)
                .build());
        moveSkystoneArms(ArmSide.FRONT, ArmStage.DROP); // Drop the stone on foundation
    }

    @Override
    public void goBackGrabDeliverSecondSkystone() {
        drive.setLogTag("goBackGrabDeliverSecondSkystone");
        drive.strafeLeft(8); // move away from the foundation, so that we do not hit bridge
        drive.skystoneGrabberFront.setPosition(0);
        drive.turnToRadians(0);
        drive.forward(forwardDistanceFromFoundation);
        // use sensor to read distance to wall, and move to 2nd skystone positions
        double expected_distance_to_wall = getExpectedDistanceToWall(opMode.getSkystonePos());
        moveForwardToDistance(expected_distance_to_wall,  10);

        moveSkystoneArms(ArmSide.FRONT, ArmStage.OPENGRABBER);
        drive.turnToRadians(0);
        // move right until 2" close to the skystone
        moveRightToDistance(2, true, 5);
        moveSkystoneArms(ArmSide.FRONT, ArmStage.GRAB);
        moveAndDropSkystoneOnFoundationWithBackwardDistance(secondBackwardDistance);
    }

    private void moveForwardToDistance(double expectedDistance, double maxMovementBack) {
        drive.setLogTag("moveForwardToDistance");
        double currentDistance = drive.getFrontDistance();
        drive.log("current distance to wall: " + currentDistance);
        drive.log("expected distance to wall: " + expectedDistance);
        if (currentDistance < 0) {
            drive.log("sensor broken. Do not move");
        }
        if (Math.abs(expectedDistance - currentDistance) < 1.5) { // do not need to move
            drive.log("Do not need to move." );
            return;
        }
        if (currentDistance > expectedDistance) {
            drive.forward(Math.min(currentDistance - expectedDistance, maxMovementBack));
        } else {
            drive.back(Math.min(expectedDistance - currentDistance, maxMovementBack));
        }
    }

    private double getExpectedDistanceToWall(SkystonePos skystonePos) {
        double offset = 0.5; // for left most pos, what's the distance we want to the wall
        switch(skystonePos) {
            case LEFT:
                return offset;
            case MIDDLE:
                return offset + 8;
            case RIGHT:
                return offset + 16;
            case UNKNOW:
                return offset + 8;
            default:
                return offset + 8;
        }
    }

    @Override
    public void moveFoundationBackAndPark() {
        drive.setLogTag("moveFoundationBackAndPark");
        // adjust position
        drive.log("followTrajectory to be away from foundation");
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeLeft(3)// away from foundation, avoid hitting it when turning
                .back(3) // forward a little for positions
                .build());
        // turn to face foundation and move
        drive.turnToRadians(Math.toRadians(-90));

        updatePose();
        // turn almost 90 degree to drag it
        drive.log("followTrajectory to turn 90 degree");
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(5) // forward
                .addMarker(0, () -> { moveFoundationServos(1); return null; }) //put mover down while moving
                .setReversed(true)
                .splineTo(new Pose2d(getX() - 25, getY() - 20, Math.toRadians(-40)))
                .setReversed(false)
                .build());
        drive.skystoneGrabberFront.setPosition(0);
        drive.turnToRadians(Math.toRadians(180)); // make another turn
        // moving away to park
        drive.log("followTrajectory to move away to park");
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(8) // push foundation to wall
                .addMarker(0, () -> { moveFoundationServos(0); return null;}) // open mover
                .back(5)
                .strafeLeft(13)
                .back(46)
                .build());
    }

    @Override
    // Used in M4 and M9
    public void fromFoundationToPark() {
        drive.strafeLeft(7);
        drive.turnToRadians(0);
        drive.forward(55);
        drive.strafeRight(4);
    }

    @Override
    // Used in M5 only
    public void turnAndMoveFoundationAndPark() {
        drive.strafeLeft(6);
        drive.back(7);
        drive.turnToRadians(Math.toRadians(-90));
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
        drive.turnToRadians(Math.toRadians(180));
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .setReversed(false)
                .forward(20)
                .build());
        drive.turnToRadians(Math.toRadians(180));
        moveFoundationServos(0);
        updatePose();
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .setReversed(true)
                .splineTo(new Pose2d(getX() - 43, getY() + 13.5, 0))
                .strafeRight(1)
                .setReversed(false)
                .build());
    }
}

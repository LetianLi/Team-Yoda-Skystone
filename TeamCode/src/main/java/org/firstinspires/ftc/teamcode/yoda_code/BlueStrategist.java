package org.firstinspires.ftc.teamcode.yoda_code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yoda_enum.ArmSide;
import org.firstinspires.ftc.teamcode.yoda_enum.ArmStage;
import org.firstinspires.ftc.teamcode.yoda_enum.SkystonePos;

public class BlueStrategist extends StrategistBase {
    private double forwardOffset = 0;
    private double[] forwardOffsetsPerPos = {10, 3, -4}; // left, middle, right
    private static double RIGHT_TO_STONE = 30;
    private double firstForwardDistance = 0;
    private double backDistanceFromFoundation = 0;
    private double secondForwardDistance = 0;

    public BlueStrategist(
            YodaMecanumDrive drive, ElapsedTime op_timer, AutonomousBase opMode) {
        super(drive, op_timer, opMode);
    }

    @Override
    public void calculateDistance() {
        drive.setLogTag("calculateDistance");
        // Depends on position, move forward/backward to stone position
        this.forwardOffset = getForwardOffset(opMode.getSkystonePos(), forwardOffsetsPerPos);
        drive.log("forwardOffset:" + forwardOffset);
        this.firstForwardDistance = 80 - forwardOffset;
        drive.log("firstForwardDistance:" + firstForwardDistance);
        // Back, we move forward 80 - forwardOffset previously, now need to go back that much,
        // plus 3 stones distance 8*3, plus some extra 8", because of loss in accuracy
        this.backDistanceFromFoundation = firstForwardDistance + 8 * 3 + 8;
        drive.log("backDistanceFromFoundation:" + backDistanceFromFoundation);
        // move forward and drop stone to foundation again, move 20" less so that we have room to
        // drop 2nd stone
        this.secondForwardDistance = this.backDistanceFromFoundation - 21;
        drive.log("secondForwardDistance:" + secondForwardDistance);
    }

    @Override
    public void grabSkyStone() {
        drive.setLogTag("grabSkyStone");

        // tell road runner of our initial position, so that it can draw position in dashboard
        drive.setPoseEstimate(new Pose2d(-33, 63, 0));
        moveSkystoneArms(ArmSide.BACK, ArmStage.OPENGRABBER);

        drive.strafeRight(RIGHT_TO_STONE); // Move right to be close to stone
        drive.turnToRadians(0); // adjust in case robot drift

        if (forwardOffset > 0) {
            drive.forward(Math.abs(forwardOffset));
        }
        else if (forwardOffset < 0){
            drive.back(Math.abs(forwardOffset));
        }
        moveSkystoneArms(ArmSide.BACK, ArmStage.GRAB);
    }

    @Override
    public void moveAndDropSkystoneOnFoundation() {
        moveAndDropSkystoneOnFoundationWithForwardDistance(this.firstForwardDistance);
    }

    private void moveAndDropSkystoneOnFoundationWithForwardDistance(double forward_distance) {
        drive.setLogTag("moveAndDropSkystoneOnFoundationWithForwardDistance");
        drive.strafeLeft(8); // move away from the stone, so that we do not hit bridge
        drive.turnToRadians(0); // adjust in case robot drift
        // Move forward, then move right to be closer to foundation
        double strafe_right_distance =  16;
        drive.log("followTrajectory: forward " + forward_distance + "+ strafeRight " + strafe_right_distance);
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(forward_distance)
                .strafeRight(strafe_right_distance)
                .build());
        moveSkystoneArms(ArmSide.BACK, ArmStage.DROP); // Drop the stone on foundation
    }

    @Override
    public void goBackGrabDeliverSecondSkystone() {
        drive.setLogTag("goBackGrabDeliverSecondSkystone");
        drive.strafeLeft(7); // move away from the foundation, so that we do not hit bridge
        drive.skystoneGrabberBack.setPosition(0);
        drive.turnToRadians(0);
        drive.back(backDistanceFromFoundation);
        // use sensor to read distance to wall, and move to 2nd skystone positions
        double distance_to_back = getExpectedDistanceToWall(opMode.getSkystonePos());
        moveBackToDistance(distance_to_back, 10);

        moveSkystoneArms(ArmSide.BACK, ArmStage.OPENGRABBER);
        drive.turnToRadians(0);
        // move right until 2" close to the skystone
        moveRightToDistance(2, true, 5);
        moveSkystoneArms(ArmSide.BACK, ArmStage.GRAB);
        moveAndDropSkystoneOnFoundationWithForwardDistance( secondForwardDistance);
    }

    private double getExpectedDistanceToWall(SkystonePos skystonePos) {
        double offset = 2; // for right most pos, what's the distance we want to the wall
        switch(skystonePos) {
            case LEFT:
                return offset + 16;
            case MIDDLE:
                return offset + 8;
            case RIGHT:
                return offset;
            case UNKNOW:
                return offset + 8;
            default:
                return offset + 8;
        }
    }

    private void moveBackToDistance(double expectedDistanceToBack, double maxMovementBack) {
        drive.setLogTag("moveBackToDistance");
        double currentDistance = drive.getBackDistance();
        drive.log("current distance to wall: " + currentDistance);
        drive.log("expected distance to wall: " + expectedDistanceToBack);
        if (currentDistance < 0) {
            drive.log("sensor broken. Do not move");
        }
        if (Math.abs(expectedDistanceToBack - currentDistance) < 1.5) { // do not need to move
            drive.log("Do not need to move." );
            return;
        }
        if (currentDistance > expectedDistanceToBack) {
            drive.back(Math.min(currentDistance - expectedDistanceToBack, maxMovementBack));
        } else {
            drive.forward(Math.min(expectedDistanceToBack - currentDistance, maxMovementBack));
        }
    }

    @Override
    public void moveFoundationBackAndPark() {
        drive.setLogTag("moveFoundationBackAndPark");
        // adjust position
        drive.log("followTrajectory to be away from foundation");
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeLeft(3)// away from foundation, avoid hitting it when turning
                .forward(1) // forward a little for positions
                .build());
        // turn to face foundation and move
        drive.turnToRadians(Math.toRadians(-90));

        updatePose();
        // turn almost 90 degree to drag it
        drive.log("followTrajectory to turn 90 degree");
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(6) // forward
                .addMarker(0.1, () -> { moveFoundationServos(1); return null; }) //put mover down while moving
                .setReversed(true)
                .splineTo(new Pose2d(getX() - 32, getY() + 15, Math.toRadians(45)))// turn
                .setReversed(false)
                .build());

        drive.turnToRadians(Math.toRadians(0)); // make another turn
        // moving away to park
        drive.log("followTrajectory to move away to park");
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(drive.getFrontDistance() - 20 + 2) // push foundation to wall
                .addMarker(0, () -> { moveFoundationServos(0); return null;}) // open mover
//                .setReversed(true)
//                .splineTo(new Pose2d(getX() - 40, getY() - 15, 0))
//
//                .back(5)// away from foundation, avoid hitting it
//                .strafeRight(5)// right to position of parking
//                .back(45) // back to parking position
                .build());
        drive.skystoneGrabberBack.setPosition(0);
        drive.turnToRadians(0);
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .setReversed(true)
                .splineTo(new Pose2d( getX() - (72 - drive.getFrontDistance()) + 15, getY() - 15, 0))
                .setReversed(false)
                .build());
    }


    @Override
    // Used in M4 and M9
    public void fromFoundationToPark() {
        drive.strafeLeft(7);
        drive.turnToRadians(0);
        drive.back(50);
        drive.strafeRight(6);
    }

    @Override
    // Used in M5 only
    public void turnAndMoveFoundationAndPark() {
        drive.strafeLeft(6);
        drive.forward(3);
        drive.turnToRadians(Math.toRadians(-90));
        drive.forward(5);
        moveFoundationServos(1);
        drive.forward(2);
        updatePose();
        opMode.sleep(500);
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .back(10)
                .setReversed(true)
                .splineTo(new Pose2d(getX() - 25, getY() + 15, 0))
                .build());
        drive.turnToRadians(0);
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

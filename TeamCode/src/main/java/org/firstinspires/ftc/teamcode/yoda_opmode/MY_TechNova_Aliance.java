package org.firstinspires.ftc.teamcode.yoda_opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.yoda_code.AutonomousBase;
import org.firstinspires.ftc.teamcode.yoda_enum.ArmSide;
import org.firstinspires.ftc.teamcode.yoda_enum.ArmStage;
import org.firstinspires.ftc.teamcode.yoda_enum.SkystonePos;
import org.firstinspires.ftc.teamcode.yoda_enum.TeamColor;

@Autonomous(group = "auto", name = "MY: TechNova Collab")
public class MY_TechNova_Aliance extends AutonomousBase {
    private double forwardOffset = 0;
    private double neg = 1;

    private double stoneY = 32;
    private double foundationY = 0;

    private double armOrder;
    private double frontArm = -5;
    private double backArm = 7;

    private Pose2d buildingZone = new Pose2d(20, 60, 0);
    private Pose2d loadingZone = new Pose2d(-12, 60, 0);
    private Pose2d centerLine = new Pose2d(0, 60, 0);

    private Pose2d startingPos;
    private double baseHeading = 0;
    private boolean reversed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        // wait for a while
        while (!isStopRequested() && op_timer.seconds() < 5) {
            telemetry.addData("Status", "Waiting to move");
            telemetry.update();
        }

        if (isStopRequested()) return;
        if (strategist != null) {

            setUpVariables();

            drive.setLogTag("main");
            logCurrentPos("*** Plan to move to pick up 1st stone");
            drive.resetLatencyTimer();

            double expected_x;
            double expected_y =  stoneY * neg;
            if ((teamColor == TeamColor.BLUE && getSkystonePos() == SkystonePos.RIGHT) ||
                (teamColor == TeamColor.RED && getSkystonePos() == SkystonePos.LEFT)) {
                expected_x = -58.5 + armOrder * neg; // get middle
            }
            else {
                expected_x = -66.5 + armOrder * neg; // get furthest
            }
            drive.log("strafeTo(" + expected_x + "," + expected_y);
            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .addMarker(0, () -> {
                        strategist.moveSkystoneArms(getArmSide(armOrder), ArmStage.LOWERARM);
                        strategist.moveSkystoneArms(getArmSide(armOrder), ArmStage.OPENGRABBER);
                        return null;})
                    .strafeTo(new Vector2d(expected_x, expected_y))
                    .addMarker(new Vector2d(expected_x, expected_y + 4 * neg), () -> { strategist.moveSkystoneArms(getArmSide(armOrder), ArmStage.CLOSEGRABBER); return null;})
                    .build());
            logError("After init move", expected_x, expected_y);

            strategist.moveSkystoneArms(getArmSide(armOrder), ArmStage.GRAB);

            moveAndDrop(1, 57, getArmSide(armOrder));

            //if (teamColor == TeamColor.BLUE) setPoseYToDistance();

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .strafeTo(new Vector2d(drive.getX(), 60 * neg))
                    .addMarker(new Vector2d(drive.getX(), 61.5 * neg), () -> {
                        strategist.resetSkystoneArms();
                        return null;})
                    .strafeTo(new Vector2d(0, 61.5 * neg))
                    .strafeLeft(10)
                    .build());
            logError("final check", 0, 60 * neg);

            sleep(30000);
        }
    }

    private void setUpVariables() {
        if (teamColor == TeamColor.RED) {
            neg = -1;
            baseHeading = Math.toRadians(180);
            reversed = true;
            startingPos = new Pose2d(-16, -61.5, baseHeading);
            foundationY = 28;
            armOrder = frontArm;
        }

        else if (teamColor == TeamColor.BLUE) {
            neg = 1;
            baseHeading = 0;
            reversed = false;
            startingPos = new Pose2d(-16, 61.5, baseHeading);
            foundationY = 32;
            armOrder = backArm;
        }
        drive.setPoseEstimate(startingPos);
        drive.resetLeftSensorToWall(startingPos.getY(), 0.1);

        buildingZone = new Pose2d(buildingZone.getX(), buildingZone.getY() * neg, baseHeading);
        loadingZone = new Pose2d(loadingZone.getX(), loadingZone.getY() * neg, baseHeading);
        centerLine = new Pose2d(centerLine.getX(), centerLine.getY() * neg, baseHeading);

        drive.turnLedOff();
    }

    private void moveAndDrop(int stoneIndex, double foundationX, ArmSide arm) {
        auto_timer.reset();
        drive.setLogTag("moveAndDrop for stone " + stoneIndex);
//        logCurrentPos("before trajectory");
        double expected_x = foundationX;
        double expected_y = (foundationY + 5) * neg;

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeTo(new Vector2d(drive.getX() + 10, 58 * neg))
                .strafeTo(new Vector2d(startingPos.getX(), 60 * neg))
                .build());
        sleep(1000);
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeTo(new Vector2d(foundationX, 60 * neg))
                .build());

        drive.strafeTo(new Vector2d(foundationX, expected_y));
        logError("after moving to foundation", expected_x, expected_y);

        Servo targetArm = (arm == ArmSide.FRONT) ? drive.skystoneArmFront : drive.skystoneArmBack;
        targetArm.setPosition(0.5); // This attempts to put arm down

        double currentDistance = drive.getRightDistance();
        drive.log("current distance right:" + currentDistance);
        if (currentDistance > 0.3) {
            drive.log("strafeRight:"+ (currentDistance - 0.3));
            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .strafeRight(currentDistance - 0.3)
                    .build());
            logError("after strafeRight", expected_x, expected_y - (currentDistance - 0.3));
        }

        Servo targetGrabber = (arm == ArmSide.FRONT) ? drive.skystoneGrabberFront : drive.skystoneGrabberBack;

        targetGrabber.setPosition(1); // open grabber
        sleep(200);
        targetArm.setPosition(0); // Put arm up

        //strategist.moveSkystoneArms(arm, ArmStage.DROP);
        drive.log("Cycle time:" + auto_timer.milliseconds());
//        logCurrentPos("after trajectory");
    }

    private void logCurrentPos(String context) {
        drive.update();
        Pose2d currentPos = drive.getPoseEstimate();

        drive.log(context + " current Position:" + currentPos.getX() + "," + currentPos.getY()
                + ", heading:" + Math.toDegrees(currentPos.getHeading()));
                //+ "\n imu:" + Math.toDegrees(drive.getRawExternalHeading()));
        ;
    }

    private ArmSide getArmSide(double side) {
        if (side == frontArm) return ArmSide.FRONT;
        else if (side == backArm) return ArmSide.BACK;
        return ArmSide.FRONT;
    }

    private double getArmOffset(ArmSide armSide) {
        switch (armSide){
            case FRONT: return frontArm;
            case BACK: return backArm;
            default: return frontArm;
        }
    }


    private void setPoseYToDistance() {
        Pose2d newPose = new Pose2d(drive.getX(), drive.getCalculatedY(startingPos.getY()), drive.getHeading());
        drive.log("Current Position: " + drive.getPoseEstimate().toString());
        drive.log("New Position: " + newPose.toString());
        drive.setPoseEstimate(newPose);
    }

}

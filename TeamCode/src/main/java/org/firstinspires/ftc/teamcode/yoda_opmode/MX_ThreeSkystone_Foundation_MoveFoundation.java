package org.firstinspires.ftc.teamcode.yoda_opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.yoda_code.AutonomousBase;
import org.firstinspires.ftc.teamcode.yoda_enum.ArmSide;
import org.firstinspires.ftc.teamcode.yoda_enum.ArmStage;
import org.firstinspires.ftc.teamcode.yoda_enum.SkystonePos;
import org.firstinspires.ftc.teamcode.yoda_enum.TeamColor;

@Autonomous(group = "auto", name = "MX: 3 Skystone->Foundation->Move it->park ")
public class MX_ThreeSkystone_Foundation_MoveFoundation extends AutonomousBase {

    private double[] blueForwardOffsets = {0, 8, 16}; // left, middle, right
    private double[] redForwardOffsets  = {24, 16, 8}; // left middle, right
    private double forwardOffset = 0;
    private ArmSide armSide = ArmSide.BACK;
    private double negativeMultiplier = 1;

    private double innerPositionOffset = 34;
    private double middlePositionOffset = 26;
    private double outterPositionOffset = 18;

    private double grabberOffset = 3.5;
    private double[] armOrder;
    private double frontArm = -3;
    private double backArm = 3.5;

    private double thirdStoneOffset = blueForwardOffsets[0];

    private Pose2d buildingZone = new Pose2d(12, 39, 0);
    private Pose2d loadingZone = new Pose2d(-12, 39, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        if (isStopRequested()) return;
        if (strategist != null) {

            setUpVariables();
            strategist.moveSkystoneArms(armSide, ArmStage.RESET);
            strategist.moveSkystoneArms(armSide, ArmStage.PREPARE);
            strategist.moveSkystoneArms(armSide, ArmStage.OPENGRABBER);

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .strafeTo(new Vector2d(-22.5 - forwardOffset + grabberOffset, 33 * negativeMultiplier))
                    .build());
            strategist.moveSkystoneArms(armSide, ArmStage.GRAB);

            moveAndDrop(innerPositionOffset);

            foundationToGrab(new Pose2d(-46.5 - forwardOffset - grabberOffset, 33 * negativeMultiplier, 0));

            moveAndDrop(middlePositionOffset);

            foundationToGrab(new Pose2d(-22.5 - thirdStoneOffset - grabberOffset, 33 * negativeMultiplier, 0));

            moveAndDrop(outterPositionOffset);

            // Move Foundation
            drive.turnToRadians(Math.toRadians(-90 * negativeMultiplier), strategist.getHeading());
            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .forward(6) // forward
                    .addMarker(0.1, () -> { strategist.moveFoundationServos(1); return null; }) //put mover down while moving
                    .setReversed(true)
                    .splineTo(new Pose2d(40, 48 * negativeMultiplier, Math.toRadians(45 * negativeMultiplier))) // turn
                    .setReversed(false)
                    .build());
            drive.turnToRadians(0, strategist.getHeading()); // make another turn

            // Push to wall and Park
            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .forward(72 - strategist.getX() + 4) // push foundation to wall
                    .addMarker(0, () -> { strategist.moveFoundationServos(0); return null;}) // open mover
                    .back(5)
                    .setReversed(true)
                    .splineTo(new Pose2d(36, 0 * negativeMultiplier, 0))
                    .build());
        }
    }

    private void setUpVariables() {
        if (teamColor == TeamColor.RED) {
            negativeMultiplier = -1;
            armSide = ArmSide.FRONT;
            grabberOffset = -3;
            drive.setPoseEstimate(new Pose2d(-32, -63, Math.toRadians(180)));

            if (getSkystonePos() == SkystonePos.LEFT) thirdStoneOffset = redForwardOffsets[1];
            else thirdStoneOffset = redForwardOffsets[2];
            forwardOffset = strategist.getForwardOffset(getSkystonePos(), redForwardOffsets);

            buildingZone = new Pose2d(buildingZone.getX(), buildingZone.getY() * negativeMultiplier, buildingZone.getHeading());
            loadingZone = new Pose2d(loadingZone.getX(), loadingZone.getY() * negativeMultiplier, loadingZone.getHeading());

            if (getSkystonePos() == SkystonePos.LEFT)        armOrder = new double[] {frontArm, frontArm, backArm};
            else if (getSkystonePos() == SkystonePos.MIDDLE) armOrder = new double[] {frontArm, backArm, backArm};
            else if (getSkystonePos() == SkystonePos.RIGHT)  armOrder = new double[] {backArm, backArm, backArm};

        }
        else if (teamColor == TeamColor.BLUE) {
            negativeMultiplier = 1;
            armSide = ArmSide.BACK;
            grabberOffset = 3.5;
            drive.setPoseEstimate(new Pose2d(-32, 63, 0));

            if (getSkystonePos() == SkystonePos.LEFT) thirdStoneOffset = blueForwardOffsets[1];
            else thirdStoneOffset = blueForwardOffsets[0];
            forwardOffset = strategist.getForwardOffset(getSkystonePos(), blueForwardOffsets);

            buildingZone = new Pose2d(buildingZone.getX(), buildingZone.getY() * negativeMultiplier, buildingZone.getHeading());
            loadingZone = new Pose2d(loadingZone.getX(), loadingZone.getY() * negativeMultiplier, loadingZone.getHeading());

            if (getSkystonePos() == SkystonePos.LEFT)        armOrder = new double[] {backArm, backArm, frontArm};
            else if (getSkystonePos() == SkystonePos.MIDDLE) armOrder = new double[] {backArm, frontArm, frontArm};
            else if (getSkystonePos() == SkystonePos.RIGHT)  armOrder = new double[] {frontArm, frontArm, frontArm};
        }
    }

    private void moveAndDrop(double foundationOffset) {
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeTo(new Vector2d(strategist.getX(), 35 * negativeMultiplier))
                .splineTo(loadingZone)
                .splineTo(buildingZone)
                .splineTo(new Pose2d(34 + foundationOffset, 32.5 * negativeMultiplier, 0))
                .build());
        strategist.moveSkystoneArms(armSide, ArmStage.DROP);
    }

    private void foundationToGrab(Pose2d stonePosition) {
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeTo(new Vector2d(strategist.getX(), 35 * negativeMultiplier))
                .addMarker(new Vector2d(strategist.getX(), 35 * negativeMultiplier), () -> { strategist.moveSkystoneArms(armSide, ArmStage.RESET); return null;})
                .setReversed(true)
                .splineTo(buildingZone)
                .splineTo(loadingZone)
                .addMarker(new Vector2d(-12, 36 * negativeMultiplier), () -> { strategist.moveSkystoneArms(armSide, ArmStage.OPENGRABBER); strategist.moveSkystoneArms(armSide, ArmStage.PREPARE); return null;})
                .splineTo(stonePosition)
                .setReversed(false)
                .build());
        strategist.moveSkystoneArms(armSide, ArmStage.GRAB);
    }
}

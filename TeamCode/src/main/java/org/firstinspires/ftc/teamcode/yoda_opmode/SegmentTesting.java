package org.firstinspires.ftc.teamcode.yoda_opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.yoda_code.AutonomousBase;
import org.firstinspires.ftc.teamcode.yoda_enum.ArmSide;
import org.firstinspires.ftc.teamcode.yoda_enum.TeamColor;

@Disabled
@Config
@TeleOp(group = "Test", name = "Segment Testing")
public class SegmentTesting extends AutonomousBase {
    public static double position;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize = false;
        initialize();
        drive.setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (isStopRequested()) return;
        while (!isStopRequested()) {
            if (gamepad2.a) {

            }
            if (gamepad2.b) {
                //strategist.moveBackToDistance(1, true, 100);
            }
            if (gamepad2.x) {

            }
            drive.skystoneArmBack.setPosition(position);

            telemetry.addData("Left Distance", drive.getLeftDistance());
            telemetry.addData("Left Distance Raw", drive.leftDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right Distance", drive.getRightDistance());
            telemetry.addData("Front Distance", drive.getFrontDistance());
            telemetry.addData("Back Distance", drive.getBackDistance());
            telemetry.update();
        }
    }
}

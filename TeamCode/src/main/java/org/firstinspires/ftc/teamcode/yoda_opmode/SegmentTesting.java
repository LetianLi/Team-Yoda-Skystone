package org.firstinspires.ftc.teamcode.yoda_opmode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.yoda_code.AutonomousBase;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

//@Disabled
@TeleOp(group = "Test", name = "Segment Testing")
public class SegmentTesting extends AutonomousBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        if (isStopRequested()) return;
        while (!isStopRequested()) {
            if (gamepad2.a) {
                strategist.moveRightToDistance(1, true, 100);
            }
            if (gamepad2.b) {
                strategist.moveBackToDistance(1, true, 100);
            }
            if (gamepad2.x) {
                drive.followTrajectorySync(drive.trajectoryBuilder()
                        .lineTo(new Vector2d(drive.getPoseEstimate().getX() + 10, drive.getPoseEstimate().getY()), new HeadingInterpolator() {
                            @Override
                            public double internalGet$core(double v, double v1) {
                                return 0;
                            }

                            @Override
                            public double internalDeriv$core(double v, double v1) {
                                return 0;
                            }

                            @Override
                            public double internalSecondDeriv$core(double v, double v1) {
                                return Math.toRadians(90);
                            }
                        })
                        .build());

            }
            telemetry.addData("Right", drive.getRightDistance());
            telemetry.addData("Back", drive.getBackDistance());
            telemetry.update();
        }
    }
}

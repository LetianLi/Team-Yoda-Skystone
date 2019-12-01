package org.firstinspires.ftc.teamcode.yoda_opmode;

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
                strategist.moveRightToDistance(1, 100);
            }
            if (gamepad2.b) {
                strategist.moveBackToDistance(1);
            }
            if (gamepad2.x) {
                drive.followTrajectorySync(drive.trajectoryBuilder()
                        .forward(10)
                        .addMarker(() -> {
                            drive.foundationMoverLeft.setPosition(1);
                            return null;
                        })
                        .back(10)
                        .build());
            }
            telemetry.addData("Right", drive.getRightDistance());
            telemetry.addData("Back", drive.getBackDistance());
            telemetry.update();
        }
    }
}

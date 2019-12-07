package org.firstinspires.ftc.teamcode.yoda_opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.yoda_code.AutonomousBase;
@Disabled
@Autonomous(group = "auto", name = "M5: 1 Skystone -> foundation, Move it, Park")
public class M5_SkystoneF_Foundation_Park extends AutonomousBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        if (isStopRequested()) return;
        if (strategist != null) {
            strategist.grabSkyStone();
            strategist.moveAndDropSkystoneOnFoundation();
            strategist.turnAndMoveFoundationAndPark();
            strategist.readyForManual();
        }
    }
}

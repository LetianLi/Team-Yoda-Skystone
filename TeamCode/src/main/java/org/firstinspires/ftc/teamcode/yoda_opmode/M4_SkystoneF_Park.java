package org.firstinspires.ftc.teamcode.yoda_opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.yoda_code.AutonomousBase;

@Autonomous(group = "auto", name = "M4: Skystone -> foundation, Park")
public class M4_SkystoneF_Park extends AutonomousBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        if (isStopRequested()) return;
        if (strategist != null) {
            strategist.GrabSkyStone();
            strategist.moveAndDropSkystoneOnFoundation();
            strategist.fromFoundationToPark();
        }
    }
}

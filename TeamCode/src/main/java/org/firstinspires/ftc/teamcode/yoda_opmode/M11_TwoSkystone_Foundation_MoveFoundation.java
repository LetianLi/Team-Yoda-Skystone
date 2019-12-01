package org.firstinspires.ftc.teamcode.yoda_opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.yoda_code.AutonomousBase;
import org.firstinspires.ftc.teamcode.yoda_enum.TeamColor;

@Autonomous(group = "auto", name = "M11: 2 Skystone->Foundation->Move it->park ")
public class M11_TwoSkystone_Foundation_MoveFoundation extends AutonomousBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        if (isStopRequested()) return;
        if (strategist != null) {
            strategist.grabSkyStone();
            strategist.moveAndDropSkystoneOnFoundation(10);
            strategist.goBackGrabDeliverSecondSkystone();
            strategist.moveFoundationBackAndPark();

        }
    }
}

package org.firstinspires.ftc.teamcode.yoda_opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.yoda_code.AutonomousBase;
@Disabled
@Autonomous(group = "auto", name = "M11: 2 Skystone->Foundation->Move it->park ")
public class M11_TwoSkystone_Foundation_MoveFoundation extends AutonomousBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        if (isStopRequested()) return;
        if (strategist != null) {
            strategist.calculateDistance();
            strategist.grabSkyStone();
            strategist.moveAndDropSkystoneOnFoundation();
            strategist.goBackGrabDeliverSecondSkystone();
            strategist.moveFoundationBackAndPark();
            strategist.readyForManual();

        }
    }
}

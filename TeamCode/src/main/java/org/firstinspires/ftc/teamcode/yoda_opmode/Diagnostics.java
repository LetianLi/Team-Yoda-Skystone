package org.firstinspires.ftc.teamcode.yoda_opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.yoda_code.AutonomousBase;
import org.firstinspires.ftc.teamcode.yoda_enum.TeamColor;

@Autonomous(group = "auto", name = "Diagnostics")
public class Diagnostics extends AutonomousBase {

    @Override
    public void runOpMode() throws InterruptedException {
        askTeamColor = false;
        teamColor = TeamColor.BLUE;
        initialize();
        if (isStopRequested()) return;
        if (strategist != null) {
            
        }
    }
}

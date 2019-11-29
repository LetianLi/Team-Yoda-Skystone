package org.firstinspires.ftc.teamcode.yoda_opmode;

// Get it from https://github.com/uhs3939/SkyStone/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opencvSkystoneDetector.java

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.yoda_code.OpencvDetector;


/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 *YES
 */
@Autonomous(name= "Opencv Test", group="Sky autonomous")
//@Disabled//comment out this line before using
public class OpencvTestOps extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpencvDetector detector = new OpencvDetector(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Values", detector.getValues());
            telemetry.addData("Skystone Pos", detector.getSkystonePos());
            telemetry.update();
            sleep(100);
        }
    }
}

package org.firstinspires.ftc.teamcode.yoda_code;

// Get it from https://github.com/uhs3939/SkyStone/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opencvSkystoneDetector.java

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.yoda_enum.SkystonePos;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;

import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

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
public class OpencvDetector {
    private HardwareMap hardwareMap;

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] defaultMidPos = {4f/8f + offsetX, 4f/8f + offsetY}; //0 = col, 1 = row
    private static float[] defaultLeftPos = {2f/8f + offsetX, 4f/8f + offsetY};
    private static float[] defaultRightPos = {6f/8f + offsetX, 4f/8f + offsetY};

    public static float[] midPos = defaultMidPos.clone();
    public static float[] leftPos = defaultLeftPos.clone();
    public static float[] rightPos = defaultRightPos.clone();
    //moves all rectangles right or left by amount. units are in ratio to monitor

    public static int movablePos = -1;
    public static double movingResolution = 0.01;

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;
    OpenCvCamera webcam;

    public OpencvDetector(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(new StageSwitchingPipeline());
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);

        /*
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC*/
        //width, height
        //width = height in this case, because camera is in portrait mode.

        midPos = defaultMidPos.clone();
        leftPos = defaultLeftPos.clone();
        rightPos = defaultRightPos.clone();
    }

    public void deactivate() {
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    public SkystonePos getSkystonePos(){
        if(valLeft == 0) return SkystonePos.LEFT;
        else if(valMid == 0) return SkystonePos.MIDDLE;
        else if(valRight == 0) return SkystonePos.RIGHT;
        else {
            return SkystonePos.UNKNOW;
        }
    }

    public String getValues() {
        return valLeft+"   "+valMid+"   "+valRight;
    }

    public double getNumberOfSkystones() {
        double numberOfSkystones = 0;
        if (valLeft == 0) numberOfSkystones++;
        if (valMid == 0) numberOfSkystones++;
        if (valRight == 0) numberOfSkystones++;
        return numberOfSkystones;
    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            if (valMid == 0) Imgproc.putText(all, "S", new Point((int)(input.cols()* midPos[0]) - 24, (int)(input.rows()* midPos[1]) - 30), 1, 5, new Scalar(255, 0, 0, 1));
            if (valLeft == 0) Imgproc.putText(all, "S", new Point((int)(input.cols()* leftPos[0]) - 24, (int)(input.rows()* leftPos[1]) - 30), 1, 5, new Scalar(255, 0, 0, 1));
            if (valRight == 0) Imgproc.putText(all, "S", new Point((int)(input.cols()* rightPos[0]) - 24, (int)(input.rows()* rightPos[1]) - 30), 1, 5, new Scalar(255, 0, 0, 1));

            //draw 3 rectangles
            Scalar movableScalar = new Scalar(255, 0, 0);
            Scalar notMovableScalar = new Scalar(0, 255, 0);
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    (movablePos == 0 || movablePos == 3)? movableScalar:notMovableScalar, 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    (movablePos == 1 || movablePos == 3)? movableScalar:notMovableScalar, 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    (movablePos == 2 || movablePos == 3)? movableScalar:notMovableScalar, 3);

            switch (stageToRenderToViewport) {
                case THRESHOLD:
                    return thresholdMat;

                case detection:
                    return all;

                case RAW_IMAGE:
                    return input;

                default:
                    return input;
            }
        }
    }
    public void incrementMovablePos() {
        movablePos += 1;
        if (movablePos > 3) movablePos = -1;
    }
    public void decrementMovablePos() {
        movablePos -= 1;
        if (movablePos < -1) movablePos = 3;
    }
    public void stopMovable() {movablePos = -1;}

    public void movePoint(double x, double y) {
        if (movablePos == 0 || movablePos == 3) {
            leftPos[0] = limit(leftPos[0] + x * movingResolution);
            leftPos[1] = limit(leftPos[1] + y * movingResolution);
        }
        if (movablePos == 1 || movablePos == 3) {
            midPos[0] = limit(midPos[0] + x * movingResolution);
            midPos[1] = limit(midPos[1] + y * movingResolution);
        }
        if (movablePos == 2 || movablePos == 3) {
            rightPos[0] = limit(rightPos[0] + x * movingResolution);
            rightPos[1] = limit(rightPos[1] + y * movingResolution);
        }
    }
    private float limit(double number) {
        while (number < 0) number += 1;
        while (number > 1) number -= 1;
        return (float) number;
    }
}

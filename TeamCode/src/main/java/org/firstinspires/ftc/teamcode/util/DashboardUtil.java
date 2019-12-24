package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */
public class DashboardUtil {
    private static final double DEFAULT_RESOLUTION = 2.0; // distance units; presumed inches
    private static final double ROBOT_WIDTH  = 16.5; // in
    private static final double ROBOT_LENGTH = 17.5; // in
    private static final double ROBOT_RADIUS = 9; // in


    public static void drawSampledPath(Canvas canvas, Path path, double resolution) {
        int samples = (int) Math.ceil(path.length() / resolution);
        double[] xPoints = new double[samples];
        double[] yPoints = new double[samples];
        double dx = path.length() / (samples - 1);
        for (int i = 0; i < samples; i++) {
            double displacement = i * dx;
            Pose2d pose = path.get(displacement);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path) {
        drawSampledPath(canvas, path, DEFAULT_RESOLUTION);
    }

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        //canvas.strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS);
        double[] leftFront  = {pose.getX() + ROBOT_LENGTH/2, pose.getY() + ROBOT_WIDTH/2};
        double[] leftRear   = {pose.getX() - ROBOT_LENGTH/2, pose.getY() + ROBOT_WIDTH/2};
        double[] rightRear  = {pose.getX() - ROBOT_LENGTH/2, pose.getY() - ROBOT_WIDTH/2};
        double[] rightFront = {pose.getX() + ROBOT_LENGTH/2, pose.getY() - ROBOT_WIDTH/2};

        leftFront = returnRotatedPoint(leftFront, pose.getHeading(), pose);
        leftRear = returnRotatedPoint(leftRear, pose.getHeading(), pose);
        rightRear = returnRotatedPoint(rightRear, pose.getHeading(), pose);
        rightFront = returnRotatedPoint(rightFront, pose.getHeading(), pose);

        canvas.strokeLine(leftFront[0], leftFront[1], leftRear[0], leftRear[1]);
        canvas.strokeLine(leftRear[0], leftRear[1], rightRear[0], rightRear[1]);
        canvas.strokeLine(rightRear[0], rightRear[1], rightFront[0], rightFront[1]);
        canvas.strokeLine(rightFront[0], rightFront[1], leftFront[0], leftFront[1]);

        canvas.strokeLine(leftFront[0], leftFront[1], rightRear[0], rightRear[1]);
        canvas.strokeLine(leftRear[0], leftRear[1], rightFront[0], rightFront[1]);

        drawHeading(canvas, pose, ROBOT_RADIUS);
//        canvas.strokeCircle(pose.getX(), pose.getY(), 2);
    }

    private static double[] returnRotatedPoint(double[] point, double angle, Pose2d pose) {
        return new double[] {(point[0] - pose.getX()) * Math.cos(angle) - (point[1] - pose.getY()) * Math.sin(angle) + pose.getX(),
                             (point[0] - pose.getX()) * Math.sin(angle) + (point[1] - pose.getY()) * Math.cos(angle) + pose.getY()};
    }

    public static void drawHeading(Canvas canvas, Pose2d pose, double radius) {
        Vector2d v = pose.headingVec().times(radius);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }
}

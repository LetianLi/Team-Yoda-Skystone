package org.firstinspires.ftc.teamcode.util;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;

import java.util.ArrayList;

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
        Vector2d leftFront  = new Vector2d(pose.getX() + ROBOT_LENGTH/2, pose.getY() + ROBOT_WIDTH/2);
        Vector2d leftRear   = new Vector2d(pose.getX() - ROBOT_LENGTH/2, pose.getY() + ROBOT_WIDTH/2);
        Vector2d rightRear  = new Vector2d(pose.getX() - ROBOT_LENGTH/2, pose.getY() - ROBOT_WIDTH/2);
        Vector2d rightFront = new Vector2d(pose.getX() + ROBOT_LENGTH/2, pose.getY() - ROBOT_WIDTH/2);

        leftFront = leftFront.rotated(pose.getHeading());
        leftRear = leftRear.rotated(pose.getHeading());
        rightRear = rightRear.rotated(pose.getHeading());
        rightFront = rightFront.rotated(pose.getHeading());
        canvas.setStrokeWidth(1);
        canvas.strokeLine(leftFront.getX(), leftFront.getY(), leftRear.getX(), leftRear.getY());
        canvas.strokeLine(leftRear.getX(), leftRear.getY(), rightRear.getX(), rightRear.getY());
        canvas.strokeLine(rightRear.getX(), rightRear.getY(), rightFront.getX(), rightFront.getY());
        canvas.strokeLine(rightFront.getX(), rightFront.getY(), leftFront.getX(), leftFront.getY());
        drawPoint(canvas, pose.vec(), 2);

        canvas.setStrokeWidth(2);
        drawHeading(canvas, pose, ROBOT_RADIUS);
//        canvas.strokeCircle(pose.getX(), pose.getY(), 2);
    }

    public static void drawPoints(Canvas canvas, ArrayList<Vector2d> list, boolean specialFirstPoint, String defaultColor, String specialColor, double radius) {
        for (int i = 0; i < list.size(); i++) {
            if (i == 0 && specialFirstPoint) {
                canvas.setFill(specialColor);
                canvas.setStroke(specialColor);
            } else if (i <= 1){
                canvas.setFill(defaultColor);
                canvas.setStroke(defaultColor);
            }
            drawPoint(canvas, list.get(i), radius);
        }
    }

    public static void drawHeading(Canvas canvas, Pose2d pose, double radius) {
        Vector2d v = pose.headingVec().times(radius);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }

    public static void drawPoint(Canvas canvas, Vector2d point, double radius) {
        canvas.fillCircle(point.getX(), point.getY(), radius);
    }
}

package org.firstinspires.ftc.teamcode.util;

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
        Vector2d leftFront  = new Vector2d(ROBOT_LENGTH/2, ROBOT_WIDTH/2);
        Vector2d leftRear   = new Vector2d(-ROBOT_LENGTH/2, ROBOT_WIDTH/2);
        Vector2d rightRear  = new Vector2d(-ROBOT_LENGTH/2, -ROBOT_WIDTH/2);
        Vector2d rightFront = new Vector2d(ROBOT_LENGTH/2, -ROBOT_WIDTH/2);

        Vector2d[] points = new Vector2d[]{leftFront, leftRear, rightRear, rightFront};

        for (int i =0; i < 4; i++) {
            points[i] = points[i].rotated(pose.getHeading());
            points[i] = points[i].plus(pose.vec());
        }

        drawShape(canvas, points);
        drawPoint(canvas, pose.vec(), 2);

        canvas.setStrokeWidth(2);
        drawHeading(canvas, pose, ROBOT_RADIUS);
//        canvas.strokeCircle(pose.getX(), pose.getY(), 2);
    }

    public static void drawRobot(Canvas canvas, double x, double y, double rad) {
        drawRobot(canvas, new Pose2d(x, y, rad));
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

    public static void drawShape(Canvas canvas, Vector2d[] points) {
        canvas.setStrokeWidth(1);
        for (int i = 0; i < points.length - 1; i++) {
            canvas.strokeLine(points[i].getX(), points[i].getY(),
                              points[i+1].getX(), points[i+1].getY());
        }
        canvas.strokeLine(points[points.length - 1].getX(), points[points.length - 1].getY(),
                          points[0].getX(), points[0].getY());
    }
}

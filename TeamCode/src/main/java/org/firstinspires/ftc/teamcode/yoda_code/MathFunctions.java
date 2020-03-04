package org.firstinspires.ftc.teamcode.yoda_code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class MathFunctions {
    /**
     * Function that wraps between -PI and PI.
     * @param angle angle that gets wrapped
     * @return angle that is wrapped
     */
    public static double radWrap(double angle) {
        double wrapped = angle % Math.toRadians(360);
        return wrapped;
    }

    /**
     * Function that returns the quadratic formula solved with +.
     * @param a a in the quadratic formula
     * @param b b in the quadratic formula
     * @param c c in the quadratic formula
     * @return x in the quadratic formula
     */
    public static double quadraticFormulaPlus(double a, double b, double c) {
        double D = Math.pow(b, 2) - 4 * a * c;
        return (-b + Math.sqrt(D)) / (2 * a);
    }

    /**
     * Function that returns the quadratic formula solved with -.
     * @param a a in the quadratic formula
     * @param b b in the quadratic formula
     * @param c c in the quadratic formula
     * @return x in the quadratic formula
     */
    public static double quadraticFormulaMinus(double a, double b, double c) {
        double D = Math.pow(b, 2) - 4 * a * c;
        return (-b - Math.sqrt(D)) / (2 * a);
    }

    /**
     * Function that returns point on line p1p2 according to t as a Vector2d
     * @param p1 the first endpoint on the line
     * @param p2 the second endpoint on the line
     * @param t the t value. 0 will be at p1, 1 will be at p2, 0.5 in the middle
     * @return the point on the line returned
     */
    public static Vector2d pointOnLine(Vector2d p1, Vector2d p2, double t) {
        return new Vector2d(p1.getX() + (p2.getX() - p1.getX()) * t,
                            p1.getY() + (p2.getY() - p1.getY()) * t);
    }

    /**
     * Function that returns point on line p1p2 according to t as a Pose2d
     * @param p1 the first endpoint on the line
     * @param p2 the second endpoint on the line
     * @param t the t value. 0 will be at p1, 1 will be at p2, 0.5 in the middle
     * @return the point on the line returned, with a heading that is wrapped
     */
    public static Pose2d pointOnLine(Pose2d p1, Pose2d p2, double t) {
        Vector2d point = pointOnLine(p1.vec(), p2.vec(), t);
        return new Pose2d(point, p1.getHeading() + (radWrap(p2.getHeading() - p1.getHeading()) * t));
    }
}

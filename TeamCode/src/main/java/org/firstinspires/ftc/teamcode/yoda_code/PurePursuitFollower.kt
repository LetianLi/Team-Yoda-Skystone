package org.firstinspires.ftc.teamcode.yoda_code

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.followers.PathFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import com.acmerobotics.roadrunner.util.NanoClock

import org.firstinspires.ftc.teamcode.yoda_code.MathFunctions.*
import java.lang.Double.isNaN

/**
 * Traditional PID controller with feedforward velocity and acceleration components to follow a trajectory. More
 * specifically, the feedback is applied to the components of the robot's pose (x position, y position, and heading) to
 * determine the velocity correction. The feedforward components are instead applied at the wheel level.
 *
 * @param searchRadius Radius of circle used to test for intersection
 * @param samplingResolution Distance between sampled points on path
 * @param constraints Mecanum constraints for movement. (eg. maxVel, maxAccel)
 * @param admissibleError admissible/satisfactory pose error at the end of each move (not used)
 * @param clock clock
 */
class PurePursuitFollower @JvmOverloads constructor(
        val searchRadius: Double = 0.25,
        val samplingResolution: Double = 0.25,
        val constraints: MecanumConstraints,
        admissibleError: Pose2d = Pose2d(),
        clock: NanoClock = NanoClock.system()
) : PathFollower(admissibleError, clock) {

    override var lastError: Pose2d = Pose2d() // Un-used

    override fun internalUpdate(currentPose: Pose2d): DriveSignal {

        val targetPose = getFollowPoint(path, currentPose, searchRadius)
        val targetVel = __________
        val targetAccel = __________

        val targetRobotVel = Kinematics.fieldToRobotVelocity(targetPose, targetVel)
        val targetRobotAccel = Kinematics.fieldToRobotAcceleration(targetPose, targetVel, targetAccel)

        return DriveSignal(targetRobotVel, targetRobotAccel)
    }

    private fun getFollowPoint(path: Path, currentPose: Pose2d, radius: Double): Pose2d {
        var p1: Pose2d
        var p2: Pose2d
        var followPoint: Pose2d = path.start()
        var s = 0.0
        while (s < path.length() - samplingResolution) {
            p1 = path[s]
            p2 = path[s + samplingResolution]

            /**
             * p1 = (x1, y1); p2 = (x2, y2); currentPose = (xc, yc); r = r;
             * Line equation is y=y1+(y2-y1)t. x=x1+(x2-x1)t. Or wraped as a whole, f(t)=p1+(p2-p1)t.
             * Where t is a constant between 0 and 1
             * Circle equation is r^2=x^2+y^2
             * Substituting line equation components into circle equation and solving for t results in a quadratic formula with:
             * a=(x1-x2)^2+(y1-y2)^2
             * b=-2[x1(x1-x2-cx)+x2*xc+y1(y1-y2-yc)+y2*yc]
             * c=(x1-xc)^2+(y1-yc)^2-r^2
             *
             * so: t=[-b+sqrt(b^2-4ac)]/2a, [-b-sqrt(b^2-4ac)]/2a
             *
             * if t<0 or 1<t></t>, discard as intersection is outside of line.
             */

            val quadraticA = Math.pow(p1.x - p2.x, 2.0) + Math.pow(p1.y - p2.y, 2.0)
            val quadraticB = -2 * (p1.x * (p1.x - p2.x - currentPose.x) + p2.x * currentPose.x + p1.y * (p1.x - p2.y - currentPose.y) + p2.y * currentPose.y)
            val quadraticC = Math.pow(p1.x - currentPose.x, 2.0) + Math.pow(p1.y - currentPose.y, 2.0) - Math.pow(radius, 2.0)

            val t1 = quadraticFormulaPlus(quadraticA, quadraticB, quadraticC)
            val t2 = quadraticFormulaMinus(quadraticA, quadraticB, quadraticC)

            val validIntersection1 = 0 <= t1 && t1 <= 1 && !isNaN(t1)
            val validIntersection2 = 0 <= t2 && t2 <= 1 && !isNaN(t2)

            val int1 = pointOnLine(p1, p2, t1)
            val int2 = pointOnLine(p1, p2, t2)

            if (validIntersection1) followPoint = int1

            if (validIntersection2) {
                if (!validIntersection1 || Math.abs(int1.x - p2.x) > Math.abs(int2.x - p2.x) || Math.abs(int1.y - p2.y) > Math.abs(int2.y - p2.y)) {
                    followPoint = int2
                }
            }
            s += samplingResolution
        }

        // special case for the very last point on the path
        if (path.length() > 0) {
            val lastPoint = path.end()

            // if we are closer than lookahead distance to the end, set it as the lookahead
            if (Math.hypot(lastPoint.x - currentPose.x, lastPoint.y - currentPose.y) <= radius) {
                return lastPoint
            }
        }

        return followPoint
    }
}

package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.math.BezierCurve;

import java.util.List;

public class TrajectoryRunner {
    private final NanoClock clock;
    private final FtcDashboard dashboard;

    private Trajectory currentTrajectory;
    private BezierCurve currentCurve;
    private double t;
    private int curveIndex;
    private List<SpatialMarker> markers;

    public static double kP = 0.28;
    public static double kCentripetal = 10.5;

    double lastCurvature;

    public TrajectoryRunner() {
        clock = NanoClock.system();
        dashboard = FtcDashboard.getInstance();
    };

    public void followTrajectory(Trajectory trajectory) {
        currentTrajectory = trajectory;
        markers = trajectory.getMarkers();
        t = 0;
        curveIndex = 0;
        currentCurve = currentTrajectory.get(0);
        lastCurvature = currentCurve.getCurvatureAt(t);
    }

    public Pose2d update(Pose2d poseEstimate) {

        t = currentCurve.getClosestTtoPoint(poseEstimate.vec());
        if (Math.abs(t - 1) <= 1e-6) {
            curveIndex += 1;
            if (curveIndex == currentTrajectory.size()) {
                curveIndex -= 1;
            } else {
                currentCurve = currentTrajectory.get(curveIndex);
            }
        }

        Vector2d reference = currentCurve.getPointAt(t);
        Vector2d derivative = currentCurve.getFirstDerivativeAt(t);
        Vector2d normDer = derivative.div(derivative.norm());

        double curvature = currentCurve.getCurvatureAt(t);
        double curvatureDeriv = (curvature - lastCurvature) / clock.seconds();
        double kAccel = 1;
        if (curvatureDeriv > 0 && curvature > 1) {
            kAccel = 0.7;
        }

        Vector2d perpDer = normDer.rotated(Math.toRadians(90)).times(kCentripetal * curvature);

        Vector2d error = reference.minus(poseEstimate.vec()).times(kP);

        // double angle = Math.asin(error.norm());

        // Vector2d pathing_power = normDer.rotated(angle);
        Vector2d pathing_power = normDer.plus(error);

        return new Pose2d(pathing_power.plus(perpDer).times(kAccel), 0);
    }
}

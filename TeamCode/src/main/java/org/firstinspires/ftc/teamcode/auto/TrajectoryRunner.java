package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dashboard.FieldDrawer;
import org.firstinspires.ftc.teamcode.math.BezierCurve;

import java.util.ArrayList;
import java.util.List;

@Config
public class TrajectoryRunner {
    private final ElapsedTime clock;
    private final FtcDashboard dashboard;

    private Trajectory currentTrajectory;
    private BezierCurve currentCurve;
    private double t;
    private int curveIndex;
    private List<SpatialMarker> markers;
    private List<StopPoint> stopPoints;
    private ArrayList<Pose2d> poseHistory;
    private ArrayList<Vector2d> turnHistory;

    public static double kP = 0.2;
    public static double kCentripetal = 0.00001;

    public static boolean uselessFlag = false;

    double lastCurvature;

    public TrajectoryRunner() {
        clock = new ElapsedTime();
        dashboard = FtcDashboard.getInstance();
    }

    public void followTrajectory(Trajectory trajectory) {
        currentTrajectory = trajectory;
        markers = trajectory.getMarkers();
        stopPoints = trajectory.getStopPoints();
        t = 0;
        curveIndex = 0;
        currentCurve = currentTrajectory.get(0);
        lastCurvature = currentCurve.getCurvatureAt(t);
        poseHistory = new ArrayList<>();
        turnHistory = new ArrayList<>();
    }

    public Pose2d update(Pose2d poseEstimate, Pose2d velocityEstimate) {
        Pose2d result;

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        Vector2d reference = currentCurve.getPointAt(t);
        Vector2d error = reference.minus(poseEstimate.vec());
        //.div(reference.minus(poseEstimate.vec()).norm())

        Vector2d derivative = currentCurve.getFirstDerivativeAt(t);
        Vector2d normDeriv = derivative.div(derivative.norm());

        double curvature = currentCurve.getCurvatureAt(t);
        // double curvatureDeriv = (Math.abs(curvature) - Math.abs(lastCurvature)) / clock.seconds();
        double accel = curvatureToForce(curvature);
        /*
        if (curvatureDeriv > curvatureDerivLimit && Math.abs(curvature) > curvatureLimit) {
            turnHistory.add(poseEstimate.vec());
            accel = kTurnAccel;
        }
         */

        Vector2d centripetalForce = normDeriv.rotated(Math.toRadians(90)).times(
                kCentripetal * Math.pow(velocityEstimate.vec().norm(), 2) * curvature
        );

        // double angle = Math.asin(error.norm());

        // Vector2d pathing_power = normDer.rotated(angle);
        Vector2d pathing_power = normDeriv.plus(error.times(kP));

        result = new Pose2d(pathing_power.plus(centripetalForce).times(accel), 0);

        t = currentCurve.getClosestTtoPoint(poseEstimate.vec());
        if (Math.abs(t - 1) <= 1e-6) {
            curveIndex += 1;
            if (curveIndex == currentTrajectory.size()) {
                curveIndex -= 1;
                result = new Pose2d(error.times(kP), 0);
            } else {
                currentCurve = currentTrajectory.get(curveIndex);
            }
        }

        if (!markers.isEmpty() && t + curveIndex >= markers.get(0).t) {
            markers.get(0).getRunnable().onMarkerReached();
            markers.remove(0);
        }

        if (!stopPoints.isEmpty() && (t + curveIndex >= stopPoints.get(0).t ||
                (Math.abs((t + curveIndex) - stopPoints.get(0).t) <= 1e-6 /*&& error.norm() <= 1*/))) {
            if (stopPoints.get(0).milliseconds >= 0) {
                stopPoints.get(0).milliseconds -= (int) clock.milliseconds();
            } else {
                stopPoints.remove(0);
            }
            result = new Pose2d(0, 0, 0);
            uselessFlag = true;
        }

        if (!result.equals(new Pose2d(0, 0, 0)) && uselessFlag) {
            curveIndex += 1;
            t = 0;
            uselessFlag = false;
        }

        poseHistory.add(poseEstimate);

        draw(fieldOverlay, poseEstimate, reference, centripetalForce, pathing_power);

        packet.put("error_norm", error.norm());
        packet.put("pathing_power_norm", pathing_power.norm());
        packet.put("x", poseEstimate.getX());
        packet.put("y", poseEstimate.getY());
        packet.put("velocity", velocityEstimate.vec().norm());
        packet.put("curvature", curvature);

        dashboard.sendTelemetryPacket(packet);

        clock.reset();

        return result;
    }

    private void draw(Canvas fieldOverlay, Pose2d poseEstimate, Vector2d reference,
                      Vector2d centripetalForce, Vector2d pathing_power) {
        FieldDrawer.drawPoint(fieldOverlay, reference, 1, "blue");

        FieldDrawer.drawRobot(fieldOverlay, poseEstimate);
        FieldDrawer.drawPoseHistory(fieldOverlay, poseHistory);
        for (Vector2d vector : turnHistory) {
            FieldDrawer.drawPoint(fieldOverlay, vector, 1, "yellow");
        }
        FieldDrawer.drawVectorFromRobot(fieldOverlay, poseEstimate.vec(), centripetalForce);
        FieldDrawer.drawVectorFromRobot(fieldOverlay, poseEstimate.vec(), pathing_power);

        for (int i = 0; i < currentTrajectory.size(); i++) {
            FieldDrawer.drawBezierCurve(
                    fieldOverlay,
                    currentTrajectory.get(i),
                    1,
                    "green"
            );
        }
    }

    private double curvatureToForce(double curvature) {
        return 0.9;
    }
}

package org.firstinspires.ftc.teamcode.dashboard;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.math.BezierCurve;

import java.util.ArrayList;
import java.util.List;

public class FieldDrawer {

    public static void drawBezierCurve(Canvas c,
                                BezierCurve curve,
                                int width,
                                String color) {
        ArrayList<Double> _xPoints = new ArrayList<>();
        ArrayList<Double> _yPoints = new ArrayList<>();

        for (int i = 0; i <= 25; i += 1) {
            Vector2d point = curve.getPointAt((double) i / 25);
            _xPoints.add(point.getX());
            _yPoints.add(point.getY());
        }

        double[] xPoints = _xPoints.stream().mapToDouble(i -> i).toArray();
        double[] yPoints = _yPoints.stream().mapToDouble(i -> i).toArray();

        c.setStrokeWidth(width).setStroke(color).strokePolyline(xPoints, yPoints);

    }

    public static void drawBezierCurvePoints(Canvas c,
                                      BezierCurve curve,
                                      double radius,
                                      String color) {
        drawPoint(c, curve.point0, radius, color);
        drawPoint(c, curve.point1, radius, color);
        drawPoint(c, curve.point2, radius, color);
        drawPoint(c, curve.point3, radius, color);
    }

    public static void drawPoint(Canvas c,
                          Vector2d point,
                          double radius,
                          String color) {
        c.setFill(color).fillCircle(point.getX(), point.getY(), radius);
    }

    public static void drawLine(Canvas c,
                                Vector2d point1,
                                Vector2d point2,
                                int width,
                                String color) {
        c.setStrokeWidth(width).setStroke(color).strokeLine(point1.getX(), point1.getY(),
                point2.getX(), point2.getY());
    }

    public static void drawVectorFromPoint(Canvas c,
                                      Vector2d point,
                                      Vector2d vector,
                                      int width,
                                      String color) {
        c.setStrokeWidth(width).setStroke(color).strokeLine(point.getX(), point.getY(),
                point.getX() + vector.getX(), point.getY() + vector.getY());
    }

    public static void drawVectorFromRobot(Canvas c,
                                           Vector2d robot,
                                           Vector2d velocity) {
        drawVectorFromPoint(c, robot, velocity.times(50), 1, "red");
    }

    public static void drawCircle(Canvas c,
                                  Vector2d center,
                                  double radius,
                                  int width,
                                  String color) {
        c.setStroke(color).setStrokeWidth(width).strokeCircle(center.getX(), center.getY(), radius);
    }

    public static void drawRectangle(Canvas c,
                                     Vector2d p1, Vector2d p2, Vector2d p3, Vector2d p4,
                                     int width,
                                     String color) {
        c.setStrokeWidth(width).setStroke(color).strokePolygon(new double[] {
                p1.getX(), p2.getX(), p3.getX(), p4.getX()
        }, new double[] {
                p1.getY(), p2.getY(), p3.getY(), p4.getY()
        });
    }

    public static void drawRobot(Canvas c, Pose2d pose) {
        double length = 13.901, width = 12.3622;

        Vector2d centerVec = pose.vec();
        Vector2d p1 = new Vector2d(length / 2, width / 2);
        Vector2d p2 = new Vector2d(-length / 2, width / 2);
        Vector2d p3 = new Vector2d(-length / 2, -width / 2);
        Vector2d p4 = new Vector2d(length / 2, -width / 2);
        p1 = p1.rotated(pose.getHeading());
        p2 = p2.rotated(pose.getHeading());
        p3 = p3.rotated(pose.getHeading());
        p4 = p4.rotated(pose.getHeading());

        drawRectangle(c, centerVec.plus(p1), centerVec.plus(p2), centerVec.plus(p3), centerVec.plus(p4), 1, "blue");
        drawPoint(c, centerVec, 1, "blue");
    }

    public static void drawPoseHistory(Canvas c, List<Pose2d> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose2d pose = poseHistory.get(i);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        c.strokePolyline(xPoints, yPoints);
    }
}

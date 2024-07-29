package org.firstinspires.ftc.teamcode.dashboard;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.math.BezierCurve;

import java.util.ArrayList;

public class FieldDrawer {

    public static void drawBezierCurve(Canvas c,
                                BezierCurve curve,
                                int width,
                                String color) {
        ArrayList<Double> _xPoints = new ArrayList<>();
        ArrayList<Double> _yPoints = new ArrayList<>();

        for (int i = 0; i <= 50; i += 1) {
            Vector2d point = curve.getPointAt((double) i / 50);
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

    public static void drawRobot(Canvas c) {
        // TODO
    }
}

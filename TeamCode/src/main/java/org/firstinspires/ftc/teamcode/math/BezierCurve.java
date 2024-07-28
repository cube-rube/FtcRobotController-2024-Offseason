package org.firstinspires.ftc.teamcode.math;

import com.acmerobotics.roadrunner.geometry.Vector2d;


public class BezierCurve {
    public Vector2d point0;
    public Vector2d point1;
    public Vector2d point2;
    public Vector2d point3;

    public BezierCurve() {}

    public BezierCurve(Vector2d point0, Vector2d point1, Vector2d point2, Vector2d point3) {
        this.point0 = point0;
        this.point1 = point1;
        this.point2 = point2;
        this.point3 = point3;
    }

    public Vector2d getPointAt(double t) {
        return point0.times(-Math.pow(t, 3) + 3 * Math.pow(t, 2) - 3 * t + 1).plus(
                point1.times(3 * Math.pow(t, 3) - 6 * Math.pow(t, 2) + 3 * t)
        ).plus(
                point2.times(-3 * Math.pow(t, 3) + 3 * Math.pow(t, 2) )
        ).plus(
                point3.times(Math.pow(t, 3))
        );
    }

    public Vector2d getFirstDerivativeAt(double t) {
        return point0.times(-3 * Math.pow(t, 2) + 6 * t - 3).plus(
                point1.times(9 * Math.pow(t, 2) - 12 * t + 3)
        ).plus(
                point2.times(-9 * Math.pow(t, 2) + 6 * t)
        ).plus(
                point3.times(3 * Math.pow(t, 2))
        );
    }

    public Vector2d getSecondDerivativeAt(double t) {
        return point0.times(-6 * t + 6).plus(
                point1.times(18 * t - 12)
        ).plus(
                point2.times(-18 * t + 6)
        ).plus(
                point3.times(6 * t)
        );
    }

    public double getCurvatureAt(double t) {
        Vector2d firstDer = getFirstDerivativeAt(t);
        Vector2d secondDer = getSecondDerivativeAt(t);
        return (firstDer.getX() * secondDer.getY() - secondDer.getX() * firstDer.getY())
                / Math.pow(firstDer.distTo(new Vector2d(0, 0)), 3);
    }

    public Vector2d getPointByIndex(int ind) {
        if (ind == 0) {
            return point0;
        } else if (ind == 1) {
            return point1;
        } else if (ind == 2) {
            return point2;
        } else if (ind == 3) {
            return point3;
        }
        return null;
    }

    public void setPointByIndex(int ind, Vector2d vector) {
        if (ind == 0) {
            point0 = vector;
        } else if (ind == 1) {
            point1 = vector;
        } else if (ind == 2) {
            point2 = vector;
        } else if (ind == 3) {
            point3 = vector;
        }
    }
}

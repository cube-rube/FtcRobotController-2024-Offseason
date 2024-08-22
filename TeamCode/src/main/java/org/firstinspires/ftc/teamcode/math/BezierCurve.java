package org.firstinspires.ftc.teamcode.math;

import androidx.annotation.NonNull;

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
    
    public double getClosestValueToPoint(Vector2d point) {
        int scans = 50;
        double min = Double.MAX_VALUE, t = 0;
        for (int i = 0; i <= scans; ++i) {
            double distance = squaredDistance(point, getPointAt((double) i / scans));
            if (distance < min) {
                min = distance;
                t = (double) i / scans;
            }
        }
        return t;
    }

    public double getClosestTtoPoint(Vector2d point) {
        int ind = 0, scans = 25;
        double min = Double.MAX_VALUE;
        for (int i = 0; i <= scans; ++i) {
            double distance = squaredDistance(point, getPointAt((double) i / scans));
            if (distance < min) {
                min = distance;
                ind = i;
            }
        }
        double t0 = Math.max((double) (ind - 1) / scans, 0);
        double t1 = Math.min((double) (ind + 1) / scans, 1);
        return localMinimum(t0, t1, point);
    }

    public double localMinimum(double minX, double maxX, Vector2d point) {
        double epsilon = 1e-10;
        double left = minX, right = maxX, mid = (right + left) / 2;
        while ((right - left) > epsilon) {
            mid = (right + left) / 2;
            double f1 = squaredDistance(point, getPointAt(mid - epsilon));
            double f2 = squaredDistance(point, getPointAt(mid + epsilon));
            if (f1 < f2) {
                right = mid;
            } else {
                left = mid;
            }
        }
        return mid;
    }

    private double squaredDistance(Vector2d point0, Vector2d point1) {
        return Math.pow(point0.getX() - point1.getX(), 2) + Math.pow(point0.getY() - point1.getY(), 2);
    }

    @NonNull
    @Override
    public String toString() {
        return "Bezier curve: " + point0.getX() + " " + point0.getY() +
                " " + point1.getX() + " " + point1.getY() +
                " " + point2.getX() + " " + point2.getY() +
                " " + point3.getX() + " " + point3.getY();
    }
}

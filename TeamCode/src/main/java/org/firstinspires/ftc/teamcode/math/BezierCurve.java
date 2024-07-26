package org.firstinspires.ftc.teamcode.math;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.Setter;

@AllArgsConstructor
@Getter
@Setter
public class BezierCurve {
    Vector2d point1;
    Vector2d point2;
    Vector2d point3;
    Vector2d point4;

    public Vector2d getPointAt(double t) {
        return point1.times(-Math.pow(t, 3) + 3 * Math.pow(t, 2) - 3 * t + 1).plus(
                point2.times(3 * Math.pow(t, 3) - 6 * Math.pow(t, 2) + 3 * t)
        ).plus(
                point3.times(-3 * Math.pow(t, 3) + 3 * Math.pow(t, 2) )
        ).plus(
                point4.times(Math.pow(t, 3))
        );
    }

    public Vector2d getFirstDerivativeAt(double t) {
        return point1.times(-3 * Math.pow(t, 2) + 6 * t - 3).plus(
                point2.times(9 * Math.pow(t, 2) - 12 * t + 3)
        ).plus(
                point3.times(-9 * Math.pow(t, 2) + 6 * t)
        ).plus(
                point4.times(3 * Math.pow(t, 2))
        );
    }

    public Vector2d getSecondDerivativeAt(double t) {
        return point1.times(-6 * t + 6).plus(
                point2.times(18 * t - 12)
        ).plus(
                point3.times(-18 * t + 6)
        ).plus(
                point4.times(6 * t)
        );
    }

    public double getCurvatureAt(double t) {
        Vector2d firstDer = getFirstDerivativeAt(t);
        Vector2d secondDer = getSecondDerivativeAt(t);
        return (firstDer.getX() * secondDer.getY() - secondDer.getX() * firstDer.getY())
                / Math.pow(firstDer.distTo(new Vector2d(0, 0)), 3);
    }
}

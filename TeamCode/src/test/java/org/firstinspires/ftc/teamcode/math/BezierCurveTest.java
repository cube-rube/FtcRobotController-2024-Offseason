package org.firstinspires.ftc.teamcode.math;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.assertj.core.data.Offset;
import org.junit.Before;
import org.junit.Test;

import static org.assertj.core.api.Assertions.assertThat;

public class BezierCurveTest {

    private BezierCurve curve;
    @Before
    public void setUp() {
        curve = new BezierCurve(
          new Vector2d(0, 0),
          new Vector2d(40, 0),
          new Vector2d(40, 40),
          new Vector2d(0, 40)
        );
    }

    @Test
    public void closestToPointOnCurve() {
        Vector2d point = new Vector2d(80, 20);
        double t = curve.getClosestTtoPoint(point);
        assertThat(t).isEqualTo(0.5, Offset.offset(1e-7));
    }
}

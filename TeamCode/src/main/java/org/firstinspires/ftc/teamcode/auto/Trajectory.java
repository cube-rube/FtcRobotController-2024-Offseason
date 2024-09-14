package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.math.BezierCurve;

import java.util.List;

public class Trajectory {
    private final Vector2d[] points;
    private final StopPoint[] stopPoints;
    private final List<SpatialMarker> markers;

    Trajectory(Vector2d[] points, List<StopPoint> stopPoints, List<SpatialMarker> markers) {
        this.points = points;
        this.stopPoints = stopPoints.toArray(new StopPoint[0]);
        this.markers = markers;
    }

    public BezierCurve get(int i) {
        return new BezierCurve(
                points[i / 3],
                points[i / 3 + 1],
                points[i / 3 + 2],
                points[i / 3 + 3]
        );
    }

    public int size() {
        return (points.length - 1) / 3;
    }

    public List<SpatialMarker> getMarkers() {
        return markers;
    }
}

package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.File;
import java.io.IOException;
import java.util.List;

public class TrajectoryBuilder {
    private final Vector2d[] points;
    private List<StopPoint> stops;
    private List<SpatialMarker> spatialMarkers;

    public TrajectoryBuilder(File file) throws IOException {
        ObjectMapper objectMapper = new ObjectMapper();
        points = objectMapper.readValue(file, Vector2d[].class);
    }

    public void addSpatialMarker(double t, MarkerRunnable runnable) {
        addSpatialMarker((it) -> t, runnable);
    }

    public void addSpatialMarker(VariableProducer producer, MarkerRunnable runnable) {
        spatialMarkers.add(new SpatialMarker(producer, runnable));
    }

    public void waitForAt(int curve, double t, int milliseconds) {
        stops.add(new StopPoint(curve, t, milliseconds));
    }

    public Trajectory build() {
        return new Trajectory(points, stops, spatialMarkers);
    }
}

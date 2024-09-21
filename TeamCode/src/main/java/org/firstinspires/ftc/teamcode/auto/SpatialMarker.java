package org.firstinspires.ftc.teamcode.auto;

public class SpatialMarker {
    public double t;
    private MarkerRunnable runnable;

    SpatialMarker(double t, MarkerRunnable runnable) {
        this.t = t;
        this.setRunnable(runnable);
    }

    public MarkerRunnable getRunnable() {
        return runnable;
    }

    public void setRunnable(MarkerRunnable runnable) {
        this.runnable = runnable;
    }
}

package org.firstinspires.ftc.teamcode.auto;

public class SpatialMarker {
    private VariableProducer producer;
    private MarkerRunnable runnable;

    SpatialMarker(VariableProducer producer, MarkerRunnable runnable) {
        this.setProducer(producer);
        this.setRunnable(runnable);
    }

    public VariableProducer getProducer() {
        return producer;
    }

    public void setProducer(VariableProducer producer) {
        this.producer = producer;
    }

    public MarkerRunnable getRunnable() {
        return runnable;
    }

    public void setRunnable(MarkerRunnable runnable) {
        this.runnable = runnable;
    }
}

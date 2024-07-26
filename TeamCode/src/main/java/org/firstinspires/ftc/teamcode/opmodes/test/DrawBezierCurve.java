package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.math.BezierCurve;

import java.lang.reflect.Array;
import java.util.ArrayList;

@Autonomous
public class DrawBezierCurve extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        BezierCurve bezierCurve = new BezierCurve(
                new Vector2d(0, 0),
                new Vector2d(0, 1),
                new Vector2d(1, 1),
                new Vector2d(1, 0)
        );

        ArrayList<Double> _xPoints = new ArrayList<>();
        ArrayList<Double> _yPoints = new ArrayList<>();

        for (int i = 0; i < 50; i += 1) {
            Vector2d point = bezierCurve.getPointAt((double) i / 50);
            _xPoints.add(point.getX());
            _yPoints.add(point.getY());
        }

        double[] xPoints = _xPoints.stream().mapToDouble(i -> i).toArray();
        double[] yPoints = _yPoints.stream().mapToDouble(i -> i).toArray();

        TelemetryPacket packet = new TelemetryPacket();

        packet.fieldOverlay().setStrokeWidth(2).setStroke("green").strokePolyline(xPoints, yPoints);

        dashboard.sendTelemetryPacket(packet);

        while (opModeIsActive()) {
            opModeIsActive();
        }
    }
}

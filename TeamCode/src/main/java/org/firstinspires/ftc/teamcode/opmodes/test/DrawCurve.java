package org.firstinspires.ftc.teamcode.opmodes.test;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.dashboard.FieldDrawer;
import org.firstinspires.ftc.teamcode.math.BezierCurve;

import java.io.File;
import java.io.IOException;

@Autonomous
public class DrawCurve extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        BezierCurve bezierCurve = new BezierCurve(
                new Vector2d(0, 0),
                new Vector2d(0, 40),
                new Vector2d(40, 40),
                new Vector2d(40, 0)
        );
        BezierCurve[] bezierCurves = {bezierCurve};

        waitForStart();

        ObjectMapper mapper = new ObjectMapper();

        String trajFilePath = String.format("%s/FIRST/data/traj.json", Environment.getExternalStorageDirectory().getAbsolutePath());

        File file = new File(trajFilePath);

        try {
            mapper.writeValue(file, bezierCurves);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket(true);
            Canvas canvas = packet.fieldOverlay();


            FieldDrawer.drawBezierCurve(canvas, bezierCurve, 1, "green");

            dashboard.sendTelemetryPacket(packet);
        }
    }
}

package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.dashboard.FieldDrawer;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.file.TrajectoryFiles;
import org.firstinspires.ftc.teamcode.math.BezierCurve;
import org.firstinspires.ftc.teamcode.math.BezierCurveLinkedList;

import java.io.IOException;

@Autonomous
@Config
public class TrajectoryFollower extends LinearOpMode {
    public static String AUTONOMOUS_NAME = "traj.json";

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        ObjectMapper objectMapper = new ObjectMapper();

        final BezierCurveLinkedList curves;
        try {
            BezierCurve[] curvesArray = objectMapper.readValue(TrajectoryFiles.loadFile(AUTONOMOUS_NAME), BezierCurve[].class);
            curves = new BezierCurveLinkedList(curvesArray);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Thread fieldDrawing = new Thread(() -> {
            while (opModeIsActive()) {
                TelemetryPacket packet = new TelemetryPacket();
                Canvas canvas = packet.fieldOverlay();
                for (BezierCurve bezierCurve : curves) {
                    FieldDrawer.drawBezierCurve(canvas, bezierCurve, 1, "green");
                }
                packet.put("Robot Position", drive.getPoseEstimate());
                dashboard.sendTelemetryPacket(packet);
            }
        });
        fieldDrawing.start();

        double t = 0;
        double tIncrement = 0.05;

        waitForStart();


        while (opModeIsActive()) {

        }



    }
}

package org.firstinspires.ftc.teamcode.opmodes.test;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dashboard.FieldDrawer;
import org.firstinspires.ftc.teamcode.file.TrajectoryFiles;
import org.firstinspires.ftc.teamcode.math.BezierCurve;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Objects;

@TeleOp(group = "Trajectory")
public class TrajectoryEditor extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        ObjectMapper objectMapper = new ObjectMapper();

        waitForStart();

        String folderPath = String.format(
                "%s/FIRST/data",
                Environment
                        .getExternalStorageDirectory()
                        .getAbsolutePath()
        );

        File dataFolder = new File(folderPath);

        String[] filesList = Objects.requireNonNull(dataFolder.list());

        String chosenFileName = null;
        int chosenFileIndex = 0;
        while (chosenFileName == null) {
            telemetry.addLine("Choose one of the following trajectories");
            telemetry.addLine("Use dpad to navigate and A to confirm");
            telemetry.addLine("-------------------------------------");
            for (int i = 0; i < filesList.length; i += 1) {
                if (chosenFileIndex == i) {
                    telemetry.addLine("* " + filesList[i]);
                } else {
                    telemetry.addLine(filesList[i]);
                }
            }
            telemetry.addLine("-------------------------------------");
            telemetry.addLine("Or create a new file by pressing B");
            telemetry.update();
            if (gamepad1.dpad_up) {
                chosenFileIndex = (chosenFileIndex - 1) % filesList.length;
            }
            if (gamepad1.dpad_down) {
                chosenFileIndex = (chosenFileIndex + 1) % filesList.length;
            }

            if (gamepad1.a) {
                chosenFileName = filesList[chosenFileIndex];
            }
            if (gamepad1.b) {
                int index = 0;
                for (String s : filesList) {
                    if (("traj" + index + ".json").equals(s)) {
                        index += 1;
                    }
                }
                chosenFileName = "traj" + index + ".json";
            }

        }
        telemetry.addLine("Chosen file " + chosenFileName);
        telemetry.addLine("Choose points with dpad and move them with right stick");
        telemetry.addLine("To add a new curve press A");
        telemetry.update();

        try {
            ArrayList<BezierCurve> curveArray = objectMapper.readValue(TrajectoryFiles.loadFile(chosenFileName),
                    new TypeReference<ArrayList<BezierCurve>>() {});

            int currentPoint = 0;
            int currentCurve = 0;


            while (opModeIsActive()) {
                TelemetryPacket packet = new TelemetryPacket();
                Canvas canvas = packet.fieldOverlay();
                for (BezierCurve bezierCurve : curveArray) {
                    FieldDrawer.drawBezierCurve(canvas, bezierCurve, 1, "green");
                    FieldDrawer.drawBezierCurvePoints(canvas, bezierCurve, 1, "red");
                }
                FieldDrawer.drawBezierCurve(canvas, curveArray.get(currentCurve), 1, "orange");
                FieldDrawer.drawPoint(canvas, curveArray.get(currentCurve).getPointByIndex(currentPoint), 1, "purple");

                dashboard.sendTelemetryPacket(packet);

                if (gamepad1.dpad_up) {
                    currentPoint += 1;
                    if (currentPoint == 4) {
                        currentPoint = 0;
                        currentCurve = (currentCurve + 1) % curveArray.size();
                    }
                }
                if (gamepad1.dpad_down) {
                    currentPoint -= 1;
                    if (currentPoint == -1) {
                        currentPoint = 3;
                        currentCurve = (currentCurve - 1) % curveArray.size();
                    }
                }

                double deltaX = -gamepad1.right_stick_y * 0.1;
                double deltaY = -gamepad1.right_stick_x * 0.1;

                Vector2d point = curveArray.get(currentCurve)
                        .getPointByIndex(currentPoint)
                        .plus(new Vector2d(deltaX, deltaY));

                curveArray.get(currentCurve).setPointByIndex(currentPoint, point);

                if (gamepad1.a) {
                    point = curveArray.get(curveArray.size() - 1).point3;
                    curveArray.add(new BezierCurve(
                            point,
                            point.plus(new Vector2d(0, 5)),
                            point.plus(new Vector2d(0, 10)),
                            point.plus(new Vector2d(0, 15))
                    ));
                }
            }


        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}

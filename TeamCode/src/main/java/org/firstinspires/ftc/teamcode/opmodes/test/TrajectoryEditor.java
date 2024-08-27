package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dashboard.FieldDrawer;
import org.firstinspires.ftc.teamcode.file.FilesystemUtil;
import org.firstinspires.ftc.teamcode.math.BezierCurve;
import org.firstinspires.ftc.teamcode.math.BezierCurveLinkedList;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.Vector;

@Config
@TeleOp(group = "Trajectory")
public class TrajectoryEditor extends LinearOpMode {

    public static String CUSTOM_FILE_NAME = null;
    public static int X = 0, Y = 0;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final NinjaGamePad gamePad1 = new NinjaGamePad(gamepad1);
    private final NinjaGamePad gamePad2 = new NinjaGamePad(gamepad2);

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        ObjectMapper objectMapper = new ObjectMapper();

        waitForStart();

        File dataFolder = FilesystemUtil.loadDataFolder();

        String[] filesList = Objects.requireNonNull(dataFolder.list());

        String chosenFileName = null;
        int chosenFileIndex = 0;
        while (chosenFileName == null && opModeIsActive()) {

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

            if (gamePad1.getDpadUp().debounced().getRise()) {
                chosenFileIndex = (((chosenFileIndex - 1) % filesList.length) + filesList.length) % filesList.length;
            }
            if (gamePad1.getDpadDown().debounced().getRise()) {
                chosenFileIndex = (chosenFileIndex + 1) % filesList.length;
            }

            if (gamePad1.getDpadDown().debounced().getRise()) {
                chosenFileName = filesList[chosenFileIndex];
            }
            if (gamePad1.getBButton().debounced().getRise()) {
                if (CUSTOM_FILE_NAME == null) {
                    int index = 0;
                    for (String s : filesList) {
                        if (("traj" + index + ".json").equals(s)) {
                            index += 1;
                        }
                    }
                    chosenFileName = "traj" + index + ".json";
                } else {
                    chosenFileName = CUSTOM_FILE_NAME;
                }
            }
        }

        ArrayList<Vector2d> points;
        File chosenFile = FilesystemUtil.loadFile(chosenFileName);

        if (chosenFile.isFile()) {
            try {
                Vector2d[] pointArray = objectMapper.readValue(chosenFile,
                        Vector2d[].class);
                points = new ArrayList<>(Arrays.asList(pointArray));
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        } else {
            points = new ArrayList<>();
            points.add(new Vector2d(0, 0));
            points.add(new Vector2d(0, 10));
            points.add(new Vector2d(0, 20));
            points.add(new Vector2d(0, 30));
        }

        int currentPoint = 0;

        while (opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();
            for (int i = 0; i < points.size() - 1; i += 3) {
                BezierCurve curve = new BezierCurve(
                        points.get(i),
                        points.get(i + 1),
                        points.get(i + 2),
                        points.get(i + 3)
                );
                FieldDrawer.drawBezierCurve(canvas, curve, 1, "green");
                FieldDrawer.drawBezierCurvePoints(canvas, curve, 1, "red");
            }
            FieldDrawer.drawPoint(canvas, points.get(currentPoint), 1, "yellow");

            dashboard.sendTelemetryPacket(packet);

            telemetry.addLine("Add new curves by pressing A");
            telemetry.addLine("Remove curves by pressing B");
            telemetry.addLine("Edit the X and Y variables and then press X to apply them to selected point");
            telemetry.addLine("Move points with right stick");
            telemetry.addLine("Change point with dpad up and down");
            telemetry.addLine("Save trajectory to file by pressing left bumper");
            telemetry.addData("point X coordinate", points.get(currentPoint).getX());
            telemetry.addData("point Y coordinate", points.get(currentPoint).getY());
            telemetry.update();

            if (gamePad1.getDpadUp().debounced().getRise()) {
                currentPoint = (currentPoint + 1) % points.size();
            }
            if (gamePad1.getDpadDown().debounced().getRise()) {
                currentPoint = (((currentPoint - 1) % points.size()) + points.size()) % points.size();
            }

            double deltaX = -gamepad1.right_stick_y * 0.1;
            double deltaY = -gamepad1.right_stick_x * 0.1;

            Vector2d point = points
                    .get(currentPoint)
                    .plus(new Vector2d(deltaX, deltaY));

            points.set(currentPoint, point);

            if (gamePad1.getAButton().debounced().getRise()) {
                point = points.get(points.size() - 1);
                points.add(point.plus(new Vector2d(0, 5)));
                points.add(point.plus(new Vector2d(0, 10)));
                points.add(point.plus(new Vector2d(0, 15)));
            }
            if (gamePad1.getBButton().debounced().getRise()) {
                points.remove(points.size() - 1);
                points.remove(points.size() - 1);
                points.remove(points.size() - 1);
            }
            if (gamePad1.getXButton().debounced().getRise()) {
                points.set(currentPoint, new Vector2d(X, Y));
            }
            if (gamePad1.getLeftBumper().debounced().getRise()) {
                try {
                    objectMapper.writeValue(chosenFile, points.toArray());
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }
}

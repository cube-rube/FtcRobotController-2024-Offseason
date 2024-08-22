package org.firstinspires.ftc.teamcode.opmodes.test;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.control.ToggledButton;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dashboard.FieldDrawer;
import org.firstinspires.ftc.teamcode.file.TrajectoryFiles;
import org.firstinspires.ftc.teamcode.math.BezierCurve;
import org.firstinspires.ftc.teamcode.math.BezierCurveLinkedList;

import java.io.File;
import java.io.IOException;
import java.util.Objects;

@Config
@TeleOp(group = "Trajectory")
public class TrajectoryEditor extends LinearOpMode {

    public static String NEW_FILE_NAME = null;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final NinjaGamePad gamePad1 = new NinjaGamePad(gamepad1);
    private final NinjaGamePad gamePad2 = new NinjaGamePad(gamepad2);

    @Override
    public void runOpMode() throws InterruptedException {
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
        boolean newFile = false;
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
                while (gamepad1.dpad_down) {
                    telemetry.update();
                }
            }

            if (gamePad1.getDpadDown().debounced().getRise()) {
                chosenFileName = filesList[chosenFileIndex];
            }
            if (gamePad1.getBButton().debounced().getRise()) {
                if (NEW_FILE_NAME == null) {
                    int index = 0;
                    for (String s : filesList) {
                        if (("traj" + index + ".json").equals(s)) {
                            index += 1;
                        }
                    }
                    chosenFileName = "traj" + index + ".json";
                    newFile = true;
                } else {
                    chosenFileName = NEW_FILE_NAME;
                }
            }

        }

        BezierCurveLinkedList curves;
        if (!newFile) {
            try {
                BezierCurve[] curvesArray = objectMapper.readValue(TrajectoryFiles.loadFile(chosenFileName),
                        BezierCurve[].class);
                curves = new BezierCurveLinkedList(curvesArray);
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        } else {
            curves = new BezierCurveLinkedList();
            curves.add(new BezierCurve(
                    new Vector2d(0, 0),
                    new Vector2d(0, 10),
                    new Vector2d(0, 20),
                    new Vector2d(0, 30)
            ));
        }

        int currentPoint = 0;

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();
            for (BezierCurve bezierCurve : curves) {
                FieldDrawer.drawBezierCurve(canvas, bezierCurve, 1, "green");
                FieldDrawer.drawBezierCurvePoints(canvas, bezierCurve, 1, "red");
            }
            FieldDrawer.drawPoint(canvas, curves.getPoint(currentPoint), 1, "yellow");

            dashboard.sendTelemetryPacket(packet);

            telemetry.addLine("Add new curves by pressing A");
            telemetry.addLine("Move points with right stick");
            telemetry.addLine("Change point with dpad up and down");
            telemetry.addLine("Save trajectory to file by pressing left bumper");
            telemetry.addData("point X coordinate", curves.getPoint(currentPoint).getX());
            telemetry.addData("point Y coordinate", curves.getPoint(currentPoint).getY());
            telemetry.update();

            if (gamePad1.getDpadUp().debounced().getRise()) {
                currentPoint = (currentPoint + 1) % curves.sizePoints();
            }
            if (gamePad1.getDpadDown().debounced().getRise()) {
                currentPoint = (((currentPoint - 1) % curves.sizePoints()) + curves.sizePoints()) % curves.sizePoints();
            }

            double deltaX = -gamepad1.right_stick_y * 0.1;
            double deltaY = -gamepad1.right_stick_x * 0.1;

            Vector2d point = curves
                    .getPoint(currentPoint)
                    .plus(new Vector2d(deltaX, deltaY));

            curves.setPoint(currentPoint, point);

            if (gamePad1.getAButton().debounced().getRise()) {
                if (curves.size() > 0) {
                    point = curves.get(curves.size() - 1).point3;
                    curves.add(new BezierCurve(
                            point,
                            point.plus(new Vector2d(0, 5)),
                            point.plus(new Vector2d(0, 10)),
                            point.plus(new Vector2d(0, 15))
                    ));
                } else {
                    curves.add(new BezierCurve(
                            new Vector2d(0, 0),
                            new Vector2d(0, 5),
                            new Vector2d(0, 10),
                            new Vector2d(0, 15)
                    ));
                }
                while (gamepad1.a) {
                    telemetry.addLine("Added a new curve");
                    telemetry.update();
                }
            }
            if (gamepad1.left_bumper) {
                try {
                    objectMapper.writeValue(TrajectoryFiles.loadFile(chosenFileName), curves.toArray());
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
                while (gamepad1.left_bumper) {
                    telemetry.addLine("Saving to file");
                    telemetry.update();
                }
            }
        }
    }
}

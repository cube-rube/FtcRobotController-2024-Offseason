package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dashboard.FieldDrawer;
import org.firstinspires.ftc.teamcode.drive.Drive;
import org.firstinspires.ftc.teamcode.file.FilesystemUtil;
import org.firstinspires.ftc.teamcode.math.BezierCurve;

import java.io.IOException;
import java.util.ArrayList;

@Autonomous
@Config
public class TrajectoryFollower extends LinearOpMode {
    public static String AUTONOMOUS_NAME = "traj5.json";
    public static double kP = 0.28;
    public static double kCentripetal = 10.5;
    public static double kAccel = 0.5;
    public static double curvatureLimit = 0.03;
    public static double curvatureDerivLimit = 1;

    private Drive drive;
    private Vector2d[] points;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        ObjectMapper objectMapper = new ObjectMapper();

        try {
            points = objectMapper.readValue(FilesystemUtil.loadFile(AUTONOMOUS_NAME), Vector2d[].class);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        Pose2d startPose = new Pose2d(points[0], Math.toRadians(0));
        ArrayList<Pose2d> poseHistory = new ArrayList<>();
        ArrayList<Vector2d> turnHistory = new ArrayList<>();
        poseHistory.add(startPose);

        drive = new Drive(hardwareMap, startPose);

        double t = 0;
        int curveIndex = 0;

        BezierCurve curve = new BezierCurve(
                points[0],
                points[1],
                points[2],
                points[3]
        );

        waitForStart();

        double lastCurvature = curve.getCurvatureAt(t);

        ElapsedTime cycle = new ElapsedTime();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket(false);
            Canvas canvas = packet.fieldOverlay();
            canvas.setAlpha(0.4);
            canvas.drawImage("/dash/powerplay.png", 0, 0, 144, 144);
            canvas.setAlpha(1.0);
            canvas.drawGrid(0, 0, 144, 144, 7, 7);

            drawTrajectory(canvas);

            Pose2d robotPosition = drive.getPoseEstimate();

            Vector2d reference = curve.getPointAt(t);
            Vector2d derivative = curve.getFirstDerivativeAt(t);
            Vector2d secondDerivative = curve.getSecondDerivativeAt(t);
            Vector2d normDer = derivative.div(derivative.norm());

            double curvature = curve.getCurvatureAt(t);
            double curvatureDeriv = (Math.abs(curvature) - Math.abs(lastCurvature)) / cycle.seconds();
            double accel = 1;
            if (curvatureDeriv > curvatureDerivLimit && Math.abs(curvature) > curvatureLimit) {
                turnHistory.add(drive.getPoseEstimate().vec());
                accel = kAccel;
            }

            Vector2d perpDer = normDer.rotated(Math.toRadians(90)).times(kCentripetal * curvature);

            Vector2d error = reference.minus(robotPosition.vec()).times(kP);

            double angle = Math.asin(error.norm());

            // Vector2d pathing_power = normDer.rotated(angle);
            Vector2d pathing_power = normDer.plus(error);

            Pose2d result = new Pose2d(pathing_power.plus(perpDer).times(accel), 0);
            drive.setPowersByPose(result);

            t = curve.getClosestTtoPoint(robotPosition.vec());
            if (Math.abs(t - 1) <= 1e-6) {
                curveIndex += 1;
                if (curveIndex == (points.length - 1) / 3) {
                    curveIndex -= 1;
                } else {
                    curve = new BezierCurve(
                            points[curveIndex * 3],
                            points[curveIndex * 3 + 1],
                            points[curveIndex * 3 + 2],
                            points[curveIndex * 3 + 3]
                    );
                }
            }

            FieldDrawer.drawPoint(canvas, reference, 1, "blue");

            FieldDrawer.drawRobot(canvas, robotPosition);
            FieldDrawer.drawPoseHistory(canvas, poseHistory);
            for (Vector2d vector : turnHistory) {
                FieldDrawer.drawPoint(canvas, vector, 1, "yellow");
            }
            FieldDrawer.drawVectorFromRobot(canvas, robotPosition.vec(), perpDer);
            FieldDrawer.drawVectorFromRobot(canvas, robotPosition.vec(), pathing_power);

            poseHistory.add(robotPosition);

            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("point x", reference.getX());
            telemetry.addData("point y", reference.getY());
            telemetry.addData("derivative x", derivative.getX());
            telemetry.addData("derivative y", derivative.getY());
            telemetry.addData("curvature", curvature);
            telemetry.addData("curvatureDeriv", curvatureDeriv);
            telemetry.addData("t", curveIndex + t);
            telemetry.addData("angle", angle);
            telemetry.addData("velocity x", pathing_power.getX());
            telemetry.addData("velocity y", pathing_power.getY());
            telemetry.addData("kAccel", kAccel);
            telemetry.update();

            cycle.reset();
        }

        while (opModeIsActive()) {
            Pose2d robotPosition = drive.getPoseEstimate();
            Vector2d reference = curve.getPointAt(t);
            Vector2d error = reference.minus(robotPosition.vec()).times(kP);
            drive.setPowersByPose(new Pose2d(error.times(kP), 0));

            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();
            drawTrajectory(canvas);

            FieldDrawer.drawPoint(canvas, reference, 1, "blue");

            FieldDrawer.drawRobot(canvas, robotPosition);
            FieldDrawer.drawVectorFromRobot(canvas, robotPosition.vec(), error);

            dashboard.sendTelemetryPacket(packet);
        }
    }

    // [ 0, 0, 0, 0, 0, 0, 0]
    private void drawTrajectory(Canvas canvas) {
        for (int i = 0; i < points.length - 1; i += 3) {
            FieldDrawer.drawBezierCurve(
                    canvas,
                    new BezierCurve(
                            points[i],
                            points[i + 1],
                            points[i + 2],
                            points[i + 3]
                    ),
                    1,
                    "green"
            );
        }
        drive.updateLocalizer();
    }
}

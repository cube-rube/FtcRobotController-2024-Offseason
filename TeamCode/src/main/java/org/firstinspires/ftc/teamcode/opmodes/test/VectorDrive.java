package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dashboard.FieldDrawer;
import org.firstinspires.ftc.teamcode.drive.Drive;

@Config
@TeleOp
public class VectorDrive extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double X = 0;
    public static double Y = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Drive drive = new Drive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        waitForStart();

        while (opModeIsActive()) {
            for (double power : drive.setPowersByPose(new Pose2d(X, Y, 0))) {
                telemetry.addLine(String.valueOf(power));
            }
            for (double power : drive.getMotorPowers()) {
                telemetry.addLine(String.valueOf(power));
            }
            drive.updateLocalizer();
            telemetry.addData("Heading", drive.getPoseEstimate().getHeading());
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();
            Vector2d pos = drive.getPoseEstimate().vec();
            FieldDrawer.drawVectorFromRobot(canvas, pos, new Vector2d(X, Y));
            FieldDrawer.drawPoint(canvas, pos, 1, "blue");
            FieldDrawer.drawRobot(canvas, drive.getPoseEstimate());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}

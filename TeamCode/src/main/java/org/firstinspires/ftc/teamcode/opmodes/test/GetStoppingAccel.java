package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Drive;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp
public class GetStoppingAccel extends LinearOpMode {

    public static double[] MOTOR_POWERS = {1, 1, 1, 1};
    public static double RUNTIME = 1;
    public static ArrayList<Double> accelValues = new ArrayList<>();;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap, new Pose2d(0, 0, 0));
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();
        resetRuntime();

        double lastVelocity = drive.getVelocityInTicks();
        ElapsedTime timer = new ElapsedTime();
        while (getRuntime() < RUNTIME) {
            drive.setMotorPowers(MOTOR_POWERS);

            double velocity = drive.getVelocityInTicks();
            double accel = (velocity - lastVelocity) / timer.seconds();
            lastVelocity = velocity;
            timer.reset();
            telemetry.addData("velocity", velocity);
            telemetry.addData("accel", accel);
            telemetry.addData("parralel", drive.getParralelVelocity());
            telemetry.addData("perpendicular", drive.getPerpendicularVelocity());
            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            drive.updateLocalizer();
            telemetry.update();
        }

        {
            double velocity = drive.getVelocityInTicks();
            double accel = (velocity - lastVelocity) / timer.seconds();
            lastVelocity = velocity;
            timer.reset();

            telemetry.addData("start_velocity", velocity);
            telemetry.addData("start_accel", accel);
            telemetry.addData("start_parralel", drive.getParralelVelocity());
            telemetry.addData("start_perpendicular", drive.getPerpendicularVelocity());
            telemetry.addData("start_x", drive.getPoseEstimate().getX());
            telemetry.addData("start_y", drive.getPoseEstimate().getY());
            telemetry.addData("start_time", getRuntime());
            drive.updateLocalizer();

            telemetry.update();
        }

        double endTime = 0;



        while (opModeIsActive()) {
            drive.setMotorPowers(0, 0, 0, 0);
            boolean zero = true;
            for (Double velocity: drive.getWheelVelocities()) {
                if (velocity != 0) {
                    zero = false;
                    break;
                }
            }
            if (zero && endTime == 0) {
                endTime = getRuntime();
            }
            double velocity = drive.getVelocityInTicks();
            double accel = (velocity - lastVelocity) / timer.seconds();
            lastVelocity = velocity;
            timer.reset();
            accelValues.add(accel);
            telemetry.addData("end_time", endTime);
            telemetry.addData("velocity", velocity);
            telemetry.addData("acceleration", accel);
            telemetry.addData("parralel", drive.getParralelVelocity());
            telemetry.addData("perpendicular", drive.getPerpendicularVelocity());
            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            drive.updateLocalizer();

            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.ode.ParameterizedODE;
import org.firstinspires.ftc.teamcode.drive.Drive;

@Config
@TeleOp
public class GetStoppingDistance extends LinearOpMode {

    public static double[] MOTOR_POWERS = {1, 1, 1, 1};
    public static double RUNTIME = 1;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap, new Pose2d(0, 0, 0));
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();
        resetRuntime();

        while (getRuntime() < RUNTIME) {
            drive.setMotorPowers(MOTOR_POWERS);
            int i = 0;
            for (Double power: drive.getWheelVelocities()) {
                telemetry.addData("wheel_velocity" + i, power.toString());
                i++;
            }
            telemetry.addData("velocity", drive.getVelocityInTicks());
            telemetry.addData("parralel", drive.getParralelVelocity());
            telemetry.addData("perpendicular", drive.getPerpendicularVelocity());
            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            drive.updateLocalizer();
            telemetry.update();
        }

        {
            int i = 0;
            for (Double power: drive.getWheelVelocities()) {
                telemetry.addData("stopping_wheel_velocity" + i, power.toString());
                i++;
            }
            telemetry.addData("start_velocity", drive.getVelocityInTicks());
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
            int i = 0;
            boolean zero = true;
            for (Double power: drive.getWheelVelocities()) {
                telemetry.addData("wheel_velocity" + i, power.toString());
                if (power != 0) {
                    zero = false;
                }
                i++;
            }
            if (zero && endTime == 0) {
                endTime = getRuntime();
            }
            telemetry.addData("end_time", endTime);
            telemetry.addData("velocity", drive.getVelocityInTicks());
            telemetry.addData("parralel", drive.getParralelVelocity());
            telemetry.addData("perpendicular", drive.getPerpendicularVelocity());
            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            drive.updateLocalizer();
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.Trajectory;
import org.firstinspires.ftc.teamcode.drive.Drive;

import java.io.IOException;

@Autonomous
public class AutoTest extends LinearOpMode {
    private Drive drive;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive = new Drive(hardwareMap);

        Trajectory traj;
        try {
            traj = drive.buildTrajectory("traj6.json").build();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        drive.followTrajectory(traj);
        waitForStart();

        while (opModeIsActive()) {
            drive.update();
        }
    }
}

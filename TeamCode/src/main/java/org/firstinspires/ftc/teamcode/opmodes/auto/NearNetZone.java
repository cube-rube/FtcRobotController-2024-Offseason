package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.Trajectory;
import org.firstinspires.ftc.teamcode.drive.Drive;

import java.io.IOException;

@Config
@Autonomous
public class NearNetZone extends LinearOpMode {
    private Drive drive;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive = new Drive(hardwareMap);

        Trajectory traj;
        try {
            traj = drive.buildTrajectory("traj6.json")
                    .waitForAt(.9, 2000)
                    .waitForAt(1.9, 2000)
                    .waitForAt(2.9, 2000)
                    .waitForAt(3.9, 2000)
                    .waitForAt(4.9, 2000)
                    .build();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        waitForStart();

        drive.followTrajectory(traj);
        while (opModeIsActive()) {
            drive.update();
        }
    }
}

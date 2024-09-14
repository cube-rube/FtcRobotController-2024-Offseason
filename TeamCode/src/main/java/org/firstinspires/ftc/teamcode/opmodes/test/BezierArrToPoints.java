package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.file.FilesystemUtil;
import org.firstinspires.ftc.teamcode.math.BezierCurve;

import java.io.IOException;

@Config
@TeleOp
public class BezierArrToPoints extends LinearOpMode {
    public static String FILENAME = "traj.json";

    @Override
    public void runOpMode() throws InterruptedException {
        ObjectMapper mapper = new ObjectMapper();
        waitForStart();
        BezierCurve[] curves;
        try {
            curves = mapper.readValue(FilesystemUtil.loadFile(FILENAME), BezierCurve[].class);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        Vector2d[] points = new Vector2d[curves.length * 3 + 1];
        int i = 0;
        for (BezierCurve curve : curves) {
            points[i] = curve.point0;
            points[i + 1] = curve.point1;
            points[i + 2] = curve.point2;
            i += 3;
        }
        points[points.length - 1] = curves[curves.length - 1].point3;
        try {
            mapper.writeValue(FilesystemUtil.loadFile(FILENAME), points);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}

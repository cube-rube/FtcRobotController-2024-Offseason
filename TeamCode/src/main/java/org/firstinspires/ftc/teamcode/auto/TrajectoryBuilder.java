package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import org.firstinspires.ftc.teamcode.math.BezierCurve;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

public class TrajectoryBuilder {
    private Vector2d[] points;

    public TrajectoryBuilder(File file) throws IOException {
        ObjectMapper objectMapper = new ObjectMapper();
        points = objectMapper.readValue(file, Vector2d[].class);
    }

    
}

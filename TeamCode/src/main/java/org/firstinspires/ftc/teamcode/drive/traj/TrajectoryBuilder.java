package org.firstinspires.ftc.teamcode.drive.traj;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.firstinspires.ftc.teamcode.drive.segments.LineSegment;
import org.firstinspires.ftc.teamcode.drive.segments.LineSegmentObject;

public class TrajectoryBuilder {
    ObjectMapper objectMapper = new ObjectMapper();

    void createJsonAndWrite() throws JsonProcessingException {
        LineSegment object = new LineSegment(new Vector2d(0, 0), new Vector2d(20, 20));

        String json = objectMapper.writeValueAsString(object.getLineSegmentObject());
        System.out.println();
    }
}

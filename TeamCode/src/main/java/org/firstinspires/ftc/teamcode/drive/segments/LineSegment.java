package org.firstinspires.ftc.teamcode.drive.segments;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import com.fasterxml.jackson.annotation.JsonTypeInfo.As;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import lombok.Getter;


public class LineSegment {
    Vector2d startPoint;
    Vector2d endPoint;
    @Getter
    LineSegmentObject lineSegmentObject;

    public LineSegment(LineSegmentObject object) {
        lineSegmentObject = object;
        startPoint = new Vector2d(object.getStartX(), object.getStartY());
        endPoint = new Vector2d(object.getEndX(), object.getEndY());
    }

    public LineSegment(Vector2d startPoint, Vector2d endPoint) {
        lineSegmentObject = new LineSegmentObject(startPoint.getX(), startPoint.getY(), endPoint.getX(), endPoint.getY());

    }
}

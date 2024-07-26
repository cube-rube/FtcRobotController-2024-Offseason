package org.firstinspires.ftc.teamcode.drive.segments;

import com.fasterxml.jackson.annotation.JsonTypeInfo;

import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
@JsonTypeInfo(include= JsonTypeInfo.As.WRAPPER_OBJECT, use= JsonTypeInfo.Id.NAME)
public class LineSegmentObject {
    private double startX;
    private double startY;
    private double endX;
    private double endY;

    public LineSegmentObject(double x, double y, double x1, double y1) {
        startX = x;
        startY = y;
        endX = x1;
        endY = y1;
    }
}


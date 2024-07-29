package org.firstinspires.ftc.teamcode.math;

import static org.assertj.core.api.Assertions.assertThat;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.junit.Test;

public class BezierCurveLinkedListTest {
    private BezierCurveLinkedList curves;

    @Test
    public void addAndGet() {
        curves = new BezierCurveLinkedList();
        BezierCurve curve = new BezierCurve(
                new Vector2d(0, 0),
                new Vector2d(0, 40),
                new Vector2d(40, 40),
                new Vector2d(40, 0)
        );
        curves.add(curve);
        assertThat(curve).usingRecursiveComparison().isEqualTo(curves.get(0));
    }

    @Test
    public void multipleAddAndGet() {
        curves = new BezierCurveLinkedList();
        BezierCurve curve1 = new BezierCurve(
                new Vector2d(0, 0),
                new Vector2d(0, 40),
                new Vector2d(40, 40),
                new Vector2d(40, 0)
        );
        BezierCurve curve2 = new BezierCurve(
                new Vector2d(40, 0),
                new Vector2d(50, 0),
                new Vector2d(60, 0),
                new Vector2d(70, 0)
        );
        curves.add(curve1);
        curves.add(curve2);
    }

    @Test
    public void constructAndGet() {
        curves = new BezierCurveLinkedList(new BezierCurve[] {
                new BezierCurve(
                        new Vector2d(0, 10),
                        new Vector2d(0, 20),
                        new Vector2d(0, 30),
                        new Vector2d(0, 40)
                ),
                new BezierCurve(
                        new Vector2d(0, 40),
                        new Vector2d(0, 50),
                        new Vector2d(0, 60),
                        new Vector2d(0, 70)
                )
        });
        assertThat(curves.get(0)).usingRecursiveComparison().isEqualTo(new BezierCurve(
                new Vector2d(0, 10),
                new Vector2d(0, 20),
                new Vector2d(0, 30),
                new Vector2d(0, 40)
        ));
        assertThat(curves.get(1)).usingRecursiveComparison().isEqualTo(new BezierCurve(
                new Vector2d(0, 40),
                new Vector2d(0, 50),
                new Vector2d(0, 60),
                new Vector2d(0, 70)
        ));
    }

    @Test
    public void constructAndIterator() {
        BezierCurve[] curvesArray = new BezierCurve[] {
                new BezierCurve(
                        new Vector2d(0, 10),
                        new Vector2d(0, 20),
                        new Vector2d(0, 30),
                        new Vector2d(0, 40)
                ),
                new BezierCurve(
                        new Vector2d(0, 40),
                        new Vector2d(0, 50),
                        new Vector2d(0, 60),
                        new Vector2d(0, 70)
                )
        };
        curves = new BezierCurveLinkedList(curvesArray);
        int i = 0;
        for (BezierCurve curve : curves) {
            assertThat(curve).usingRecursiveComparison().isEqualTo(curvesArray[i]);
            i++;
        }
    }

    @Test
    public void constructAndToArray() {
        BezierCurve[] curvesArray = new BezierCurve[] {
                new BezierCurve(
                        new Vector2d(0, 10),
                        new Vector2d(0, 20),
                        new Vector2d(0, 30),
                        new Vector2d(0, 40)
                ),
                new BezierCurve(
                        new Vector2d(0, 40),
                        new Vector2d(0, 50),
                        new Vector2d(0, 60),
                        new Vector2d(0, 70)
                )
        };
        curves = new BezierCurveLinkedList(curvesArray);
        BezierCurve[] newCurvesArray = curves.toArray();
        assertThat(newCurvesArray).usingRecursiveComparison().isEqualTo(curvesArray);
    }
}

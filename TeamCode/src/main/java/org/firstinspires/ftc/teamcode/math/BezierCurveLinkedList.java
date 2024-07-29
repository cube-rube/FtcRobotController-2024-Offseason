package org.firstinspires.ftc.teamcode.math;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.Iterator;

public class BezierCurveLinkedList implements Iterable<BezierCurve> {
    Node head;
    private int size = 0;

    static class Node {
        Vector2d point0;
        Vector2d point1;
        Vector2d point2;
        Vector2d nextPoint;
        Node next;

        Node(BezierCurve curve) {
            point0 = curve.point0;
            point1 = curve.point1;
            point2 = curve.point2;
            nextPoint = curve.point3;
            next = null;
        }

        Vector2d getPointByIndex(int i) throws IndexOutOfBoundsException {
            if (i == 0) {
                return point0;
            } else if (i == 1) {
                return point1;
            } else if (i == 2) {
                return point2;
            } else if (i == 3 && next == null) {
                return nextPoint;
            } else if (i == 3 && nextPoint == null) {
                return next.point0;
            } else {
                throw new IndexOutOfBoundsException("Invalid index " + i + " (max is 3)");
            }
        }

        void setPointByIndex(int i, Vector2d vector) throws IndexOutOfBoundsException {
            if (i == 0) {
                point0 = vector;
            } else if (i == 1) {
                point1 = vector;
            } else if (i == 2) {
                point2 = vector;
            } else if (i == 3 && next == null) {
                nextPoint = vector;
            } else if (i == 3 && nextPoint == null) {
                next.point0 = vector;
            } else {
                throw new IndexOutOfBoundsException("Invalid index " + i + " (max is 3)");
            }
        }
    }

    public BezierCurveLinkedList() {
        head = null;
    }

    public BezierCurveLinkedList(BezierCurve[] arr) {
        head = new Node(arr[0]);

        Node last = head;
        for (int i = 1; i < arr.length; ++i) {
            last.nextPoint = null;
            last.next = new Node(arr[i]);
            last = last.next;
        }

        size = arr.length;
    }

    @NonNull
    @Override
    public Iterator<BezierCurve> iterator() {
        return new Iterator<BezierCurve>() {
            Node cursor = BezierCurveLinkedList.this.head;
            @Override
            public boolean hasNext() {
                return cursor != null;
            }

            @Override
            public BezierCurve next() {
                if (cursor.next != null) {
                    BezierCurve data = new BezierCurve(
                            cursor.point0,
                            cursor.point1,
                            cursor.point2,
                            cursor.next.point0
                    );
                    cursor = cursor.next;
                    return data;
                }
                if (cursor.nextPoint != null) {
                    BezierCurve data = new BezierCurve(
                            cursor.point0,
                            cursor.point1,
                            cursor.point2,
                            cursor.nextPoint
                    );
                    cursor = null;
                    return data;
                }
                return null;
            }
        };
    }

    public void insert(BezierCurve curve) {
        Node newNode = new Node(curve);
        if (head == null) {
            head = newNode;
        } else {
            Node last = head;
            while (last.next != null) {
                last.nextPoint = null;
                last = last.next;
            }

            last.next = newNode;
        }
        size += 1;
    }

    public void add(BezierCurve curve) {
        if (head == null) {
            head = new Node(curve);
        } else {
            Node last = head;
            while (last.next != null) {
                last = last.next;
            }
            last.next = new Node(curve);
            last.nextPoint = null;
        }
        size += 1;
    }

    public BezierCurve get(int i) throws IndexOutOfBoundsException {
        Node last = head;
        while (i > 0) {
            last = last.next;
            i--;
        }
        if (last.nextPoint != null) {
            return new BezierCurve(
                    last.point0,
                    last.point1,
                    last.point2,
                    last.nextPoint
            );
        }
        if (last.next != null) {
            return new BezierCurve(
                    last.point0,
                    last.point1,
                    last.point2,
                    last.next.point0
            );
        }
        throw new IndexOutOfBoundsException("Invalid index " + i + ", size is " + size);
    }

    public Vector2d getPoint(int i) {
        Node last = head;
        while (i > 3) {
            last = last.next;
            i -= 3;
        }
        return last.getPointByIndex(i);
    }

    public void setPoint(int i, Vector2d vector) {
        Node last = head;
        while (i > 3) {
            last = last.next;
            i -= 3;
        }
        last.setPointByIndex(i, vector);
    }

    public void remove(int i) {
        if (i == 0) {
            head = head.next;
        }

        Node prev = head, last = head;

        while (i > 0) {
            prev = last;
            last = last.next;
            i--;
        }
        assert prev != null;
        prev.next = last.next;
        size -= 1;
    }

    public BezierCurve[] toArray() {
        BezierCurve[] res = new BezierCurve[size];
        Node last = head;
        for (int i = 0; i < size; ++i) {
            if (i == size - 1) {
                res[i] = new BezierCurve(
                        last.point0,
                        last.point1,
                        last.point2,
                        last.nextPoint
                );
            } else {
                res[i] = new BezierCurve(
                        last.point0,
                        last.point1,
                        last.point2,
                        last.next.point0
                );
            }
            last = last.next;
        }
        return res;
    }

    public int size() {
        return size;
    }

    public int sizePoints() {
        int size = this.size * 3;
        if (size > 0) {
            size += 1;
        }
        return size;
    }
}

package org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;

public class PointArray {

    ArrayList<Point> pointList = new ArrayList<>();

    public void appendPoint(double x, double y) {
        Point point = new Point(x, y);
        pointList.add(point);
    }

    public void clear() {
        pointList.clear();
    }

    public Point get(int index) {
        return pointList.get(index);
    }

    public int length() {
        return pointList.size();
    }
}

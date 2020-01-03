package org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;

public class Path {

    ArrayList<Point> pointList = new ArrayList<>();

    public void addPoint(double x, double y, double angle) {
        Point point = new Point(x, y, angle);
        pointList.add(point);
    }

    public void addPoint(double x, double y) {
        Point point = new Point(x, y);
        pointList.add(point);
    }

    public void addPoint(Point point) {
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

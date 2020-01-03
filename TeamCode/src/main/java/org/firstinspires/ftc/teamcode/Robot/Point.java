package org.firstinspires.ftc.teamcode.Robot;

public class Point {

    public double x;
    public double y;
    public double angle;
    public boolean hasAngle = false;

    public Point() {

    }

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point(double x, double y, double angle) {
        hasAngle = true;
        this.angle = angle;
        this.x = x;
        this.y = y;
    }

    public void setX(double x) {
        this.x = x;
    }
    public void setY(double y) {
        this.y = y;
    }
    public void setPoint(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public void setPoint(double x, double y, double angle) {
        hasAngle = true;
        this.angle = angle;
        this.x = x;
        this.y = y;
    }
    public void setPoint(Point point) {
        this.x = point.x;
        this.y = point.y;
    }

    public Point add(Point point) {
        Point newPoint = new Point(this.x+point.x, this.y+point.y);
        return newPoint;
    }

    public Point subtract(Point point) {
        Point newPoint = new Point(this.x-point.x, this.y-point.y);
        return newPoint;
    }
}

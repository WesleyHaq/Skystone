package org.firstinspires.ftc.teamcode.Robot;

public class Line {

    double intercept;
    double slope;

    public Line(double slope, double intercept) {
        this.slope = slope;
        this.intercept = intercept;
    }
    public Line(Point point1, Point point2) {
        slope = (point2.y-point1.y)/(point2.x-point1.x);
        intercept = point1.y - (point1.x * slope);
    }
    public Line(Point point, double slope) {
        this.slope = slope;
        intercept = point.y - (point.x * slope);
    }

    public void setLine(double slope, double intercept) {
        this.slope = slope;
        this.intercept = intercept;
    }
    public void setLine(Point point1, Point point2) {
        slope = (point2.y-point1.y)/(point2.x-point1.x);
        intercept = point1.y - (point1.x * slope);
    }
    public void setLine(Point point, double slope) {
        this.slope = slope;
        intercept = point.y - (point.x * slope);
    }

    public Point intersection(Line line) {
        /*y = ax + b;
        y = cx + d;
        (ax + b) - (cx + d) = 0;
        ax + b - cx - d = 0;
        ax - cx = d - b
        x(a - c) = d - b
        x = (d - b) / (a - c)
         */
        double x = (intercept - line.intercept) / (line.slope - slope);
        double y = slope * x + intercept;

        Point point = new Point(x, y);

        return point;
    }
}

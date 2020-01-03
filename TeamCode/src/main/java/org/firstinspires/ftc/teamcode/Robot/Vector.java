package org.firstinspires.ftc.teamcode.Robot;

public class Vector extends Point {
    public double magnitude;
    public double direction;
    
    public Vector() {
        
    }
    
    public Vector(double magnitude, double direction) {
        this.magnitude = magnitude;
        this.direction = direction;
    }
    
    public Vector(double magnitude, double direction, double x, double y) {
        this.magnitude = magnitude;
        this.direction = direction;
        this.x = x;
        this.y = y;
    }

    public void setMagnitude(double magnitude) {
        this.magnitude = magnitude;
    }
    public void setDirection(double direction) {
        this.direction = direction;
    }
}

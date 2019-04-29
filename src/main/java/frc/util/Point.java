package frc.util;

import frc.robot.Constants;

/**
 * Point
 */
public class Point {

    private double x, y, theta;

    public Point(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public Point(Point p) {
        this.x = p.x;
        this.y = p.y;
        this.theta = p.theta;
    }

    public Point(double x, double y) {
        this(x, y, 0);
    }

    public Point translate(double delta, double angle) {
        double dx = delta * Math.cos(angle);
        double dy = delta * Math.sin(angle);
        return new Point(x + dx, y + dy);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setTheta(double theta) {
        this.theta = theta;
    }

    public Point clone() {
        return new Point(this);
    }

    public double hypot() {
        return Math.hypot(x,y);
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof Point)) {
            return false;
        }
        Point p = (Point) obj;
        boolean equals = true;
        equals &= Util.withinEpsilon(x, p.x, Constants.Units.EPSILON);
        equals &= Util.withinEpsilon(y, p.y, Constants.Units.EPSILON);
        equals &= Util.withinEpsilon(theta, p.theta, Constants.Units.EPSILON);
        return equals;
    }

    @Override
    public String toString() {
        return x + ", " + y + ", " + theta + "\n";
    }

    public void rotateByAngleDegrees(double rotation) {
        rotation = Math.toRadians(rotation);

        double x_ = x * Math.cos(rotation) - y * Math.sin(rotation);
        double y_ = x * Math.sin(rotation) + y * Math.cos(rotation);

        this.x = x_;
        this.y = y_;
    }

}
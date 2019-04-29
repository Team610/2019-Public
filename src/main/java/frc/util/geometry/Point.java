package frc.util.geometry;

/**
 * Point
 */
public class Point {

    private double x;
    private double y;

    public Point() {

    }

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point(final Point p) {
        this(p.x, p.y);
    }

    public Point translate(double dist, double theta) {
        double dx = dist * Math.cos(theta);
        double dy = dist * Math.sin(theta);
        return new Point(x + dx, y + dy);
    }

    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }

    @Override
    public Point clone() {
        return new Point(x, y);
    }

}
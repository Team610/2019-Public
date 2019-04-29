package frc.util.geometry;

/**
 * Pose
 */
public class Pose extends Point {

    private double theta;

    public Pose(double theta) {
        this.theta = theta;
    }

    public Pose(final Point p, double theta) {
        super(p.getX(), p.getY());
        this.theta = theta;
    }

    public Pose(double x, double y, double theta) {
        super(x, y);
        this.theta = theta;
    }

    public double getTheta() {
        return this.theta;
    }

    @Override
    public Pose clone() {
        return new Pose(getX(), getY(), theta);
    }

    @Override
    public String toString() {
        return "x: " + getX() + " y: " + getY() + " theta: " + getTheta();
    }

}
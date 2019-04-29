package frc.util;

/**
 * DriveSignal
 */
public class DriveSignal {

    private double left;
    private double right;

    public DriveSignal(double left, double right) {
        this.left = left;
        this.right = right;
    }
    
    public double getLeft() {
        return this.left;
    }

    public double getRight() {
        return this.right;
    }

    @Override
    public String toString() {
        return "Left: " + left + " Right: " + right;
    }

}
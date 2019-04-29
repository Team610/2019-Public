package frc.util;

/**
 * Util
 */
public class Util {

    public static double clamp(double value, double min, double max) {
        if (value < min) {
            value = min;
        }
        if (value > max) {
            value = max;
        }
        return value;
    }

    public static boolean withinRange(double number, double min, double max) {
        return min < number && number < max;
    }

    public static boolean withinEpsilon(double value, double desired, double epsilon) {
        if (Math.abs(desired - value) <= epsilon) {
            return true;
        }
        return false;
    }

    public static double deadband(double value, double threshhold) {
        if (Math.abs(value) < threshhold) {
            value = 0;
        }
        return value;
    }

}
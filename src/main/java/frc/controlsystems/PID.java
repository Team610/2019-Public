package frc.controlsystems;

import frc.util.Util;

public class PID {

  private double kP;
  private double kI;
  private double kD;
  private double min;
  private double max;

  private double lastError;
  private double totError;

  public PID(double kP, double kI, double kD) {
    this(kP, kI, kD, Double.MIN_VALUE, Double.MAX_VALUE);
  }

  /**
   * @param kP  the propotionality constant
   * @param kI  the integral constant
   * @param kD  the derivitive constant
   * @param min the minimum value from getValue
   * @param max the maximum value from getValue
   */
  public PID(double kP, double kI, double kD, double min, double max) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.min = min;
    this.max = max;
  }

  public void reset() {
    lastError = 0;
    totError = 0;
  }

  /**
   * @param cur  the current value
   * @param goal the desired value
   * @return the value from the equation p*kP+i*kI+d*kD
   */
  public double getUncappedValue(double cur, double goal) {
    double p = goal - cur;
    double i = totError;
    double d = p - lastError;
    lastError = p;
    totError += p;

    return p * kP + i * kI + d * kD;
  }

  /**
   * @param cur  the current value
   * @param goal the desired value
   * @return the capped value from the equation p*kP+i*kI+d*kD
   */
  public double getValue(double cur, double goal) {
    double out = getUncappedValue(cur, goal);
    out = Util.clamp(out, min, max);
    return out;
  }

  public double getMin() {
    return min;
  }

  public double getMax() {
    return max;
  }

  public void setP(double p) {
    this.kP = p;
  }

  public void setI(double i) {
    this.kI = i;
  }

  public void setD(double d) {
    this.kD = d;
  }

  public void setMax(double max) {
    this.max = max;
  }

}
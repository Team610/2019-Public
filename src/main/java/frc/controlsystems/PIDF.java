package frc.controlsystems;

import frc.util.Util;

public class PIDF extends PID {

  private double kF;

  public PIDF(double kP, double kI, double kD, double kF) {
    super(kP, kI, kD);
    this.kF = kF;
  }

  public PIDF(double kP, double kI, double kD, double kF, double min, double max) {
    super(kP, kI, kD, min, max);
    this.kF = kF;
  }

  @Override
  public double getUncappedValue(double cur, double goal) {
      double out = super.getUncappedValue(cur, goal);
      out += goal * kF;
      return out;
  }

  @Override
  public double getValue(double cur, double goal) {
    double out = getUncappedValue(cur, goal);
    out = Util.clamp(out, getMin(), getMax());
    return out;
  }

}
package frc.util;

import java.util.ArrayList;

/**
 * RollingAverage
 */
public class RollingAverage {

    private int size;
    private ArrayList<Double> buffer;

    public RollingAverage (int size) {
        buffer = new ArrayList<>(size);
        this.size = size;
    }

    public void addData(double item) {
        if(buffer.size() >= size) {
            buffer.remove(0);
        }
        buffer.add(item);
    }

    public double getAverage() {
        double sum = 0;
        for(double d : buffer) {
            sum += d;
        }
        return sum / buffer.size();
    }

    public int size() {
        return buffer.size();
    }

    public boolean isUnderFilled() {
        return buffer.size() < size;
    }
    
    public void clear() {
        buffer.clear();
    }

}
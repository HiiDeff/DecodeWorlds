package org.firstinspires.ftc.teamcode.util.filter;

public class MovingAverage {
    private final double[] window;
    private final int size;
    private int index = 0;
    private int count = 0;
    private double sum = 0.0;

    public MovingAverage(int size) {
        this.size = size;
        this.window = new double[size];
    }

    public void add(double value) {
        if (count == size) {
            sum -= window[index];
        } else {
            count++;
        }

        window[index] = value;
        sum += value;

        index = (index + 1) % size;
    }

    public double get() {
        if (count == 0) return 0.0;
        return sum / count;
    }

    public int getCount() {
        return count;
    }

    public void reset() {
        for (int i = 0; i < size; i++) window[i] = 0.0;
        index = 0;
        count = 0;
        sum = 0.0;
    }
}

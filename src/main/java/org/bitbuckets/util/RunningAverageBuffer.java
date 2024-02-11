package org.bitbuckets.util;

public class RunningAverageBuffer {

    final double[] buffer;

    public RunningAverageBuffer(int size) {
        this.buffer = new double[size];

        for (int i = 0; i < size; i++) {
            buffer[i] = Double.MAX_VALUE;
        }
    }

    public void insert(double measurement_primeUnits) {
        //push every index back

        double[] newBuffer = new double[buffer.length];

        newBuffer[0] = measurement_primeUnits;
        //ignore the last element
        System.arraycopy(buffer, 0, newBuffer, 1, buffer.length - 1);
        System.arraycopy(newBuffer, 0, buffer, 0, buffer.length);

    }

    public double average_primeUnits() {
        double average = 0;

        for (double d : buffer) {
            average += d;
        }

        return average / buffer.length;
    }

    public boolean isAtAverage(double measurement_primeUnits, double delta_primeUnits) {
        return Math.abs(measurement_primeUnits - average_primeUnits()) < delta_primeUnits;
    }

}

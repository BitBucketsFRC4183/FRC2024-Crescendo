package org.bitbuckets.util;
/**
public class ShooterCalculator {

    public record ReturnType(double angle_rotations, double velocity_metersPerSecond);

    final double yMax_meters;


    public ShooterCalculator(double yMaxMeters) {
        yMax_meters = yMaxMeters;
    }

    public ReturnType calculate() {

        double yo;
        double vyo = Math.sqrt(yMax_meters - yo); // yo = height of note exit (constant?)
        double g = 9.8;
        double xo = ; // horizontal distance - Note exit from robot to speaker midpoint
        double tmaxy = vyo / g;
        double vxo = -xo / tmaxy;



        return new ReturnType(Math.atan(vyo / vxo),Math.sqrt(Math.pow(vyo, 2) + Math.pow(vxo, 2)));
    }


}

 **/
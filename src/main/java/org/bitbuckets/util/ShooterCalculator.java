package org.bitbuckets.util;

public class ShooterCalculator {

    /*
    Note Specifications
    Inside Diameter: 10 in. --> 0.2540 m
    Material: Coated Foam
    Outside Diameter: 14 in --> 0.3556 m
    Thickness: 2 in. --> 5.08 cm
    Weight: 8.3 Oz +/- 0.2 Oz --> 0.24 kg +/- 0.01 kg
     */

    public ShooterCalculator(double mass, double area_cross_section, double note_radius, double air_density, double drag_coefficient, double magnus_coefficient, double vx, double vy) {
        this.mass = mass;
        this.area_cross_section = area_cross_section;
        this.note_radius = note_radius;
        this.air_density = air_density;
        this.drag_coefficient = drag_coefficient;
        this.magnus_coefficient = magnus_coefficient;
        Vx = vx;
        Vy = vy;
    }

    double mass;
    double area_cross_section;
    double note_radius;
    double air_density;
    double drag_coefficient;
    double magnus_coefficient;
    double Vx;
    double Vy;


    public double calculateAccelerationX ()
    {
        double accelerationx = -(Math.sqrt(Vx*Vx+Vy*Vy)/mass)*(drag_coefficient*Vx+magnus_coefficient*Vy);
        return accelerationx;
    }

    public double calculateAccerationY()
    {
        double accelerationy = (Math.sqrt(Vx*Vx+Vy*Vy)/mass)*(magnus_coefficient*Vx-drag_coefficient*Vy) + 9.81;
        return accelerationy;
    }
    public void velocityFinalX(){
       double Vxf = Vx;
    }
    public void trapezoidalIntegrationX()
    {

    }
    public void calculateAngle (double speaker_height, double speaker_distance)
    {

    }

}


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
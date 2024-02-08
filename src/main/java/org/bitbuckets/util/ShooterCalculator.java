package org.bitbuckets.util;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class ShooterCalculator implements Subsystem {

    /*
    Note Specifications
    Inside Diameter: 10 in. --> 0.2540 m
    Material: Coated Foam
    Outside Diameter: 14 in --> 0.3556 m
    Thickness: 2 in. --> 5.08 cm
    Weight: 8.3 Oz +/- 0.2 Oz --> 0.24 kg +/- 0.01 kg
     */

    public ShooterCalculator(ShooterCalculatorComponent shooterCalculatorComponent, double mass, double area_cross_section, double note_radius, double air_density, double drag_coefficient, double magnus_coefficient, double Vx, double Vy) {
        this.shooterCalculatorComponent = shooterCalculatorComponent;
        this.mass = mass;
        this.area_cross_section = area_cross_section;
        this.note_radius = note_radius;
        this.air_density = air_density;
        this.drag_coefficient = drag_coefficient;
        this.magnus_coefficient = magnus_coefficient;
        this.Vx = Vx;
        this.Vy = Vy;
    }

    ShooterCalculatorComponent shooterCalculatorComponent;
    double mass;
    double area_cross_section;
    double note_radius;
    double air_density;
    double drag_coefficient;
    double magnus_coefficient;
    double Vx;
    double Vy;
    double x = shooterCalculatorComponent.speaker_distance();
    double y = shooterCalculatorComponent.speaker_height();
    double Ax = -(Math.sqrt(Vx*Vx+Vy*Vy)/mass)*(drag_coefficient*Vx+magnus_coefficient*Vy);
    double Ay = (Math.sqrt(Vx*Vx+Vy*Vy)/mass)*(magnus_coefficient*Vx-drag_coefficient*Vy) + 9.81;

    /*

    public double calculateAccelerationX ()
    {
        double accelerationx = -(Math.sqrt(Vx*Vx+Vy*Vy)/mass)*(drag_coefficient*Vx+magnus_coefficient*Vy);
        return accelerationx;
    }

    public double calculateAccelerationY()
    {
        double accelerationy = (Math.sqrt(Vx*Vx+Vy*Vy)/mass)*(magnus_coefficient*Vx-drag_coefficient*Vy) + 9.81;
        return accelerationy;
    }

     */


    @Override
    public void periodic(){
        updateVelocityAndAcceleration();
        double angle = calculateAngle();

    }

    public void updateVelocityAndAcceleration()
    {
        if (y > 0) {
            double Vxf = Vx + Ax;
            double Vyf = Vy + Ay;

            x = trapezoidalIntegrationOfPosition(x, Vx, Vxf);
            y = trapezoidalIntegrationOfPosition(y, Vy, Vyf);

            Vx = Vxf;
            Vy = Vyf;

            double v = findMagnitude(Vx, Vy);

            Ax = -(v/mass) * (drag_coefficient*Vx+magnus_coefficient*Vy);
            Ay = 9.81 + (v/mass) * (magnus_coefficient*Vx-drag_coefficient*Vy);

        }
    }

    public double findMagnitude(double x_component, double y_component)
    {
        return Math.sqrt(x_component*x_component + y_component*y_component);
    }

    public double trapezoidalIntegrationOfPosition(double initial_position, double initial_velocity, double final_velocity)
    {
        return initial_position + (initial_velocity + final_velocity)/2;
    }
    public double calculateAngle ()
    {
        double v = findMagnitude(Vx, Vy);
        double a = findMagnitude(Ax, Ay);

        return Math.atan(1/(Math.sqrt((2* shooterCalculatorComponent.speaker_height()*9.81*9.81+9.81*v*v)/(2*a*Math.pow(v,4)+9.81*v*v))));
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
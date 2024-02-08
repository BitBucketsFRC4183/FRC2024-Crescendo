package org.bitbuckets.shooter;

import java.awt.*;
//import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.List;

public class ShooterCalculation {
    double mass;
    double area_cross_section;
    double note_radius;
    double air_density;
    double drag_coefficient;
    double magnus_coefficient;
    double x;
    double y;
    double holdX;
    double holdY;
    double slope;
    int goodPath;
    int goodGoodPath;
    int badPath;
    int numCurves;
    int numVelocities;
    double Vxi;
    double Vyi;
    double Ax;
    double Ay;
    double initialX;
    double initialY;

    double theta;
    double initialTheta;
    double finalTheta;
    List<Double> thetaList = new ArrayList<>();
    double delta_theta;
    double bestTheta;

    double velocity;
    double initialVelocity;
    double finalVelocity;
    List<Double> velocityList = new ArrayList<>();
    double delta_velocity;
    double bestVelocity;

    //double time;
    final double gravity = 386.1;
    double dt;

    //ugly code below but oh well
    double startX = 32; //20, 4.72
    double startY = 361.6566; //400, 598.12
    double hypotenuse = 20.567;
    double endX = startX + hypotenuse * Math.cos(28.93*Math.PI/180);
    double endY = startY - hypotenuse * Math.sin(28.93*Math.PI/180);
    Line2D backSpeaker = new Line2D.Double(50,374.6316,50,452.6316);
    Line2D topSpeaker = new Line2D.Double(startX, startY, endX, endY);
    Line2D frontSpeaker = new Line2D.Double(startX, startY, startX, startY+8.1);
    Line2D openingLine = new Line2D.Double(startX, startY+8.1, 50, 374.6316);
    Line2D backLine = new Line2D.Double(79.25,300,79.25,452.6316);


    public void setup() {
        //config
        mass = 0.25;
        area_cross_section = 0.4;
        note_radius = 0.1778;
        air_density = 1.225;
        x = 0; // get from vision
        y = 400; // get from vision
        initialX = 0; //starting X of shooter
        initialY = 400; //starting Y of shooter
        goodPath = 0;
        goodGoodPath = 0;
        badPath = 0;
        bestTheta = 0;

        //tune
        drag_coefficient = 0;
        magnus_coefficient = 0;
        numCurves = 60; //8
        numVelocities = 1000;

        initialTheta = 15;
        finalTheta = 75;
        theta = initialTheta;
        delta_theta = (finalTheta - initialTheta)/numCurves;

        initialVelocity = 100; //170
        finalVelocity = 900; //340
        velocity = initialVelocity;
        delta_velocity = (finalVelocity - initialVelocity)/numVelocities;

        Vxi = getVxi(theta);
        Vyi = getVyi(theta);
        //time = 0;
        dt = 0.001; //0.02
    }

    public void resetSetup()
    {
        x = initialX;
        y = initialY;
        Vxi = getVxi(theta);
        Vyi = getVyi(theta);
        Ax = -(Math.sqrt(Vxi * Vxi + Vyi * Vyi) / mass) * (drag_coefficient * Vxi + magnus_coefficient * Vyi);
        Ay = (Math.sqrt(Vxi * Vxi + Vyi * Vyi) / mass) * (magnus_coefficient * Vxi - drag_coefficient * Vyi) + gravity;
        //time = 0;
        goodPath = 0;
        badPath = 0;
    }

    public double getVxi(double theta) {
        return velocity * Math.cos(Math.toRadians(theta));
    }

    public double getVyi(double theta) {
        return -velocity * Math.sin(Math.toRadians(theta));
    }

    public double setXAcceleration(double Vxi, double Vyi){
        Ax = -(Math.sqrt(Vxi * Vxi + Vyi * Vyi) / mass) * (drag_coefficient * Vxi + magnus_coefficient * Vyi);
        return Ax;
    }
    public double setYAcceleration(double Vxi, double Vyi){
        Ay = (Math.sqrt(Vxi * Vxi + Vyi * Vyi) / mass) * (magnus_coefficient * Vxi - drag_coefficient * Vyi) + gravity;
        return Ay;
    }

    public void updateVelocityAndAcceleration() {
        double dVx1 = Ax * dt;
        double dVy1 = Ay * dt;

        double dVx2 = dt * setXAcceleration(dVx1/2+Vxi, dVy1/2+Vyi);
        double dVy2 = dt * setYAcceleration(dVx1/2+Vxi, dVy1/2 + Vyi);

        x += dt*(dVx2/2+Vxi);
        y += dt*(dVy2/2+Vyi);

        Vxi += dVx2;
        Vyi += dVy2;

        Ax = setXAcceleration(Vxi,Vyi);
        Ay = setYAcceleration(Vxi,Vyi);
    }


    public void drawOneCurve() {
        for (int i = 0; i < 1000; i++) {
            //g.fill(new Ellipse2D.Double(x, y, 1, 1));
            //time += dt;
            holdX = x;
            holdY = y;
            updateVelocityAndAcceleration();
            Line2D newLine = new Line2D.Double(holdX, holdY, x, y);
            slope = (y - holdY)/(x - holdX);
            if (newLine.intersectsLine(frontSpeaker) || newLine.intersectsLine(topSpeaker) || newLine.intersectsLine(backSpeaker) || (newLine.intersectsLine(backLine) && slope <= 0)) {
                badPath += 1;
                //g.setColor(Color.WHITE);
            }
            else if (newLine.intersectsLine(openingLine) && badPath == 0) {
                goodPath += 1;
                //g.setColor(Color.RED);
            }
            if (goodPath >= 1 && newLine.intersectsLine(backLine) && slope > 0){
                goodGoodPath += 1;
                //g.setColor(Color.PINK);
            }
        }
        /*
        if (badPath >= 1){
            g.setColor(Color.WHITE);
        }
        else if (goodPath == 0){
            g.setColor(Color.WHITE);
        }
         */
    }

    public void draw() {
        for (int i = 0; i < numVelocities; i++) {
            for (int k = 0; k < numCurves; k++) {
                drawOneCurve();
                //g.setColor(Color.WHITE);
                //if (goodPath >= 1) {
                //thetaList.add(theta); //use if you want middle angle
                //bestTheta = theta; //use if you want max angle
                //}
                resetSetup();
                theta += delta_theta;
            }
            if (goodGoodPath >= 1){
                velocityList.add(velocity);
            }
            goodGoodPath = 0;
            velocity += delta_velocity;
        }

        /*
        g.setColor(Color.GREEN);
        g.draw(backSpeaker); //vertical wall
        g.setColor(Color.BLUE);
        g.draw(topSpeaker);
        g.setColor(Color.YELLOW);
        g.draw(frontSpeaker); //shorter vertical wall
        g.setColor(Color.ORANGE);
        g.draw(openingLine); //opening line
        g.setColor(Color.CYAN);
        g.draw(backLine);
         */

        double sumVelocity = 0;
        for (Double aDouble : velocityList) {
            sumVelocity += aDouble;
        }
        bestVelocity = sumVelocity/velocityList.size();

        velocity = bestVelocity;
        theta = initialTheta;
        for (int k = 0; k < numCurves; k++) {
            drawOneCurve();
            if (goodPath >= 1) {
                thetaList.add(theta); //use if you want middle angle
                //bestTheta = theta; //use if you want max angle
            }
            resetSetup();
            theta += delta_theta;
        }
        double sumTheta = 0;
        for (Double add : thetaList) {
            sumTheta += add;
        }
        bestTheta = sumTheta/thetaList.size();

        //System.out.println("Best theta: " + bestTheta);
        //System.out.println("Best velocity: " + bestVelocity);
        //System.out.println("Best theta: " + sum/thetaList.size());
        //System.out.println("Current velocity: " + velocity);
        //System.out.println("run!");

        //stupid fix to solve bug with it constantly running the same thing
        //setup();
    }
}

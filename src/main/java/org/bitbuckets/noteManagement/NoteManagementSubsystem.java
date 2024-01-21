package org.bitbuckets.noteManagement;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Subsystem;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.hardware.IRotationalMotor;

public class NoteManagementSubsystem implements Subsystem {

    final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(4, 3);
    final IRotationalMotor motorOne;
    final IRotationalMotor motorTwo;
    final IRotationEncoder absoluteEncoder;

    public NoteManagementSubsystem(IRotationalMotor motorOne, IRotationalMotor motorTwo, IRotationEncoder absoluteEncoder) {
        this.motorOne = motorOne;
        this.motorTwo = motorTwo;
        this.absoluteEncoder = absoluteEncoder;
    }

    @Override
    public void periodic() {

    }

    public void setMotorRotationalSpeeds(double leftMotorSpeed_rotationsPerSecond, double rightMotorSpeed_rotationsPerSecond) {
        double leftVoltage = feedforward.calculate(leftMotorSpeed_rotationsPerSecond);
        double rightVoltage = feedforward.calculate(rightMotorSpeed_rotationsPerSecond);

        motorOne.setToVoltage(leftVoltage);
        motorTwo.setToVoltage(rightVoltage);

        motorOne.angularVelocity_mechanismRotationsPerSecond();
        motorTwo.angularVelocity_mechanismRotationsPerSecond();
    }

    public void setAllMotorsToVoltage(double voltage) {
        motorOne.setToVoltage(voltage);
        motorTwo.setToVoltage(voltage);
    }

    public void maintainSpeed(double motorOne_rotationsPerSecond, double motorTwo_rotationsPerSecond) {
        setMotorRotationalSpeeds(motorOne_rotationsPerSecond, motorTwo_rotationsPerSecond);
    }

    public boolean hasReachedSpeeds(double motorOneSpeeds, double motorTwoSpeeds) {

        boolean oneAtSpeed = motorOne.angularVelocity_mechanismRotationsPerSecond() >= motorOneSpeeds;
        boolean twoAtSpeed = motorTwo.angularVelocity_mechanismRotationsPerSecond() >= motorTwoSpeeds;

        return oneAtSpeed && twoAtSpeed;

    }
}

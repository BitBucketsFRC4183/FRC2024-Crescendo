package org.bitbuckets.shooter;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.util.AbsoluteEncoderComponent;
import xyz.auriium.mattlib2.IPeriodicLooped;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.hardware.IRotationalController;
import xyz.auriium.mattlib2.hardware.IRotationalMotor;
import xyz.auriium.yuukonstants.exception.ExplainedException;

import java.util.Optional;

public class ShooterSubsystem implements Subsystem, IPeriodicLooped {

    // converts desired velocity into voltage
    final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(-3.9555685080898364,7.794219410765281);
    public final IRotationalMotor leftMotor; //TODO find a way to not use public here (linearFFGenRoutine)
    public final IRotationalMotor rightMotor;
    final IRotationalController angleMotor;
    final IRotationEncoder absoluteEncoder;
    final ShooterComponent shooterComponent;
    final AbsoluteEncoderComponent encoderComponent;
    final IRotationEncoder velocityEncoder;

    final ShooterCalculation shooterCalculation = new ShooterCalculation();

    public ShooterSubsystem(IRotationalMotor leftMotor, IRotationalMotor rightMotor, IRotationalController angleMotor, IRotationEncoder absoluteEncoder, ShooterComponent shooterComponent, AbsoluteEncoderComponent encoderComponent, IRotationEncoder velocityEncoder) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.angleMotor = angleMotor;
        this.absoluteEncoder = absoluteEncoder;
        this.shooterComponent = shooterComponent;
        this.encoderComponent = encoderComponent;
        this.velocityEncoder = velocityEncoder;
        mattRegister();
        register();
    }

    @Override
    public Optional<ExplainedException> verifyInit() {
        absoluteEncoder.forceRotationalOffset(
                encoderComponent.offset_mechanismRotations()
        );

        angleMotor.forceRotationalOffset(
                absoluteEncoder.angularPosition_normalizedMechanismRotations()
        );

        return Optional.empty();
    }


    @Override
    public void periodic() {

    }


    public void setMotorRotationalSpeeds(double leftMotorSpeed_rotationsPerSecond, double rightMotorSpeed_rotationsPerSecond) {
        double leftVoltage = feedforward.calculate(leftMotorSpeed_rotationsPerSecond);
        double rightVoltage = feedforward.calculate(rightMotorSpeed_rotationsPerSecond);

        leftMotor.setToVoltage(leftVoltage);
        rightMotor.setToVoltage(rightVoltage);

        leftMotor.angularVelocity_mechanismRotationsPerSecond();
        rightMotor.angularVelocity_mechanismRotationsPerSecond();
    }

    public void setAllMotorsToVoltage(double voltage) {
        leftMotor.setToVoltage(voltage);
        System.out.println("this is the left motor");
        rightMotor.setToVoltage(voltage);
        System.out.println("this is the right motor");
    }

    /*
    public void moveToAngle(double angle_degrees) {
        double mechanism_rotations = angle_degrees/360d;
        moveToRotation(mechanism_rotations);
    }

     */

    public void moveToRotation(double mechanism_rotations) {
        angleMotor.controlToNormalizedReference(mechanism_rotations);

    }

    public void setPivotMotorToVoltage(double voltage) {
        angleMotor.setToVoltage(voltage);
    }

    public void setZeroAngle() {

        double offset = absoluteEncoder.angularPosition_normalizedMechanismRotations();
        angleMotor.forceRotationalOffset(offset);
    }


    public boolean hasReachedSpeeds(double leftSpeeds, double rightSpeeds) {
        boolean leftAtSpeed = leftMotor.angularVelocity_mechanismRotationsPerSecond() >= leftSpeeds;
        boolean rightAtSpeed = rightMotor.angularVelocity_mechanismRotationsPerSecond() >= rightSpeeds;

        return leftAtSpeed && rightAtSpeed;
    }

    public static boolean isWithinDeadband(double deadband, double target, double actual) {
        return actual >= target - deadband || actual <= target + deadband;
    }

    public boolean hasReachedAngle(double angle_mechanismRotations) {
        double currentPos_mechRot = angleMotor.angularPosition_normalizedMechanismRotations();

        return isWithinDeadband(shooterComponent.deadband_mechanismRotations(), angle_mechanismRotations, currentPos_mechRot);
    }

    public double getPivotAnglePosition_normalizedMechanismRotations() {
        return angleMotor.angularPosition_normalizedMechanismRotations();
    }

    public double calculateMinimalAngleForSpeaker() {
        return 0;
    }
    public double calculateMaximalAngleForSpeaker() {
        return 0.16472536666;
    }

    public double getOptimalAngle_degrees(double startX, double startY){
        shooterCalculation.setup();
        shooterCalculation.initialX = 32-startX; //distance from top front speaker
        shooterCalculation.initialY = 452.6316 - startY; //distance from ground
        shooterCalculation.draw();
        return shooterCalculation.bestTheta;
    }

    public double getOptimalVelocity_inchesPerSecond (double startX, double startY){
        shooterCalculation.setup();
        shooterCalculation.initialX = 32-startX; //distance from top front speaker
        shooterCalculation.initialY = 452.6316 - startY; //distance from ground
        shooterCalculation.draw();
        return shooterCalculation.bestVelocity;
    }




}

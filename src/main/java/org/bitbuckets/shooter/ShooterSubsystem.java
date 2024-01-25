package org.bitbuckets.shooter;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.util.CommonEncoderComponent;
import xyz.auriium.mattlib2.IPeriodicLooped;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.hardware.IRotationalController;
import xyz.auriium.mattlib2.hardware.IRotationalMotor;
import xyz.auriium.yuukonstants.exception.ExplainedException;

import java.util.Optional;

public class ShooterSubsystem implements Subsystem, IPeriodicLooped {

    // converts desired velocity into voltage
    final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(4,3);
    public final IRotationalMotor leftMotor; //TODO find a way to not use public here (linearFFGenRoutine)
    final IRotationalMotor rightMotor;
    final IRotationalController angleMotor;
    final IRotationEncoder absoluteEncoder;
    final ShooterComponent shooterComponent;
    final CommonEncoderComponent encoderComponent;
    final DigitalInput noteSensor;

    public ShooterSubsystem(IRotationalMotor leftMotor, IRotationalMotor rightMotor, IRotationalController angleMotor, IRotationEncoder absoluteEncoder, ShooterComponent shooterComponent, CommonEncoderComponent encoderComponent) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.angleMotor = angleMotor;
        this.absoluteEncoder = absoluteEncoder;
        this.shooterComponent = shooterComponent;
        this.encoderComponent = encoderComponent;
        this.noteSensor = new DigitalInput(shooterComponent.dio());

        mattRegister();
        register();
    }

    @Override
    public Optional<ExplainedException> verifyInit() {
        absoluteEncoder.forceRotationalOffset(
                encoderComponent.getAbsoluteEncoderOffset()
        );

        angleMotor.forceRotationalOffset(
                absoluteEncoder.angularPosition_normalizedMechanismRotations()
        );

        return Optional.empty();
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
        rightMotor.setToVoltage(voltage);
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

    public void moveToMechanismPosition(double mechanism_positions) {
        angleMotor.controlToNormalizedReference(mechanism_positions);
    }

    public void setZeroAngle() {

        double offset = absoluteEncoder.angularPosition_normalizedMechanismRotations();
        angleMotor.forceRotationalOffset(offset);
    }

    public void intake() {
        moveToRotation(0.125);
        // rotate wheels in the other direction
        setMotorRotationalSpeeds(-4000,-4000);
    }

    // needs velocity pid in mattlib to be added first to work; wip
    public void maintainSpeed(double leftWheel_rotationsPerSecond, double rightWheel_rotationsPerSecond) {
        setMotorRotationalSpeeds(leftWheel_rotationsPerSecond, rightWheel_rotationsPerSecond);
    }

    @Override
    public void periodic() {

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

    //returns true if note is in the sensor
    public boolean getNoteState(){
        return !noteSensor.get(); // get() returns true if circuit open
    }


}

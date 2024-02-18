package org.bitbuckets.shooter;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.util.AbsoluteEncoderComponent;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.hardware.IRotationalController;
import xyz.auriium.mattlib2.loop.IMattlibHooked;
import xyz.auriium.yuukonstants.exception.ExplainedException;

public class ShooterSubsystem implements Subsystem, IMattlibHooked {

    // converts desired velocity into voltage
    final SimpleMotorFeedforward feedforward;
    public final IRotationalController leftMotor; //TODO find a way to not use public here (linearFFGenRoutine)
    public final IRotationalController rightMotor;
    final IRotationalController angleMotor;
    final IRotationEncoder pivotEncoder;
    final IRotationEncoder shooterVelocityEncoder;

    final ShooterComponent shooterComponent;
    final AbsoluteEncoderComponent encoderComponent;


    

    public ShooterSubsystem(IRotationalController leftMotor, IRotationalController rightMotor, IRotationalController angleMotor, IRotationEncoder pivotEncoder, ShooterComponent shooterComponent, AbsoluteEncoderComponent encoderComponent, IRotationEncoder shooterVelocityEncoder) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.angleMotor = angleMotor;
        this.pivotEncoder = pivotEncoder;
        this.shooterComponent = shooterComponent;
        this.encoderComponent = encoderComponent;
        this.shooterVelocityEncoder = shooterVelocityEncoder;

        feedforward = new SimpleMotorFeedforward(shooterComponent.ks(),shooterComponent.kv());

        mattRegister();
        register();
    }

    @Override
    public ExplainedException[] verifyInit() {
        pivotEncoder.forceRotationalOffset(
                encoderComponent.offset_mechanismRotations()
        );

        angleMotor.forceRotationalOffset(
                pivotEncoder.angularPosition_normalizedMechanismRotations()
        );

        return new ExplainedException[0];
    }


    @Override
    public void periodic() {

    }


    public void setMotorRotationalSpeeds(double leftMotorSpeed_rotationsPerSecond, double rightMotorSpeed_rotationsPerSecond) {
        double leftVoltage = feedforward.calculate(leftMotorSpeed_rotationsPerSecond);
        double rightVoltage = feedforward.calculate(rightMotorSpeed_rotationsPerSecond);
       //RobotContainer.SHOOTER_TUNING.voltage(leftVoltage);
       //RobotContainer.SHOOTER_TUNING.voltage(rightVoltage);

        leftMotor.setToVoltage(leftVoltage);
        rightMotor.setToVoltage(rightVoltage);
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

    public void setZeroAngle() {

        double offset = pivotEncoder.angularPosition_normalizedMechanismRotations();
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




}

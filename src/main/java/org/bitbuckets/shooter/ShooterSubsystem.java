package org.bitbuckets.shooter;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.util.EncoderComponent;
import xyz.auriium.mattlib2.IPeriodicLooped;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.hardware.IRotationalController;
import xyz.auriium.mattlib2.hardware.IRotationalMotor;
import xyz.auriium.mattlib2.hardware.IRotationalPositionControl;
import yuukonstants.exception.ExplainedException;

import java.util.Optional;

public class ShooterSubsystem implements Subsystem, IPeriodicLooped {

    // converts desired velocity into voltage
    final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(4,3);
    final IRotationalMotor leftMotor;
    final IRotationalMotor rightMotor;
    final IRotationalController angleMotor;
    final IRotationEncoder absoluteEncoder;
    final ShooterComponent shooterComponent;
    final EncoderComponent encoderComponent;

    public ShooterSubsystem(IRotationalMotor leftMotor, IRotationalMotor rightMotor, IRotationalController angleMotor, IRotationEncoder absoluteEncoder, ShooterComponent shooterComponent, EncoderComponent encoderComponent) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.angleMotor = angleMotor;
        this.absoluteEncoder = absoluteEncoder;
        this.shooterComponent = shooterComponent;
        this.encoderComponent = encoderComponent;

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

    public void setAllMotorsToVoltage(double voltage)
    {
        leftMotor.setToVoltage(voltage);
        rightMotor.setToVoltage(voltage);
    }

    /*
    public void moveToAngle(double angle_degrees) {
        double mechanism_rotations = angle_degrees/360d;
        moveToRotation(mechanism_rotations);
    }

     */

    public void moveToRotation(double mechanism_rotations)
    {
        angleMotor.controlToNormalizedReference(mechanism_rotations);

    }

    public void setPivotMotorToVoltage(double voltage) {
        angleMotor.setToVoltage(voltage);
    }

    public void moveToMechanismPosition(double mechanism_positions)
    {
        angleMotor.controlToNormalizedReference(mechanism_positions);
    }

    public void setZeroAngle()
    {

        double offset = absoluteEncoder.angularPosition_normalizedMechanismRotations();
        angleMotor.forceRotationalOffset(offset);
    }

    public void intake() {
        moveToRotation(0.125);
        // rotate wheels in the other direction
        setMotorRotationalSpeeds(-4000,-4000);
    }

    // needs velocity pid in mattlib to be added first to work; wip
    public void maintainSpeed(double leftWheel_rotationsPerSecond, double rightWheel_rotationsPerSecond)
    {
        setMotorRotationalSpeeds(leftWheel_rotationsPerSecond, rightWheel_rotationsPerSecond);

    }

    @Override
    public void periodic() {

    }

    public boolean hasReachedSpeeds(double speed1, double speed2) {

        if (leftMotor.angularVelocity_mechanismRotationsPerSecond() >= speed1) {
            return true;
        } else {
            return false;
        }
    }

    public boolean hasReachedAngle(double angle_mechanismRotations) {
        if (angleMotor.angularPosition_normalizedMechanismRotations() == angle_mechanismRotations) {
            return true;
        } else {
            return false;
        }
    }

    public double getPivotAnglePosition_normalizedMechanismRotations() {
        return angleMotor.angularPosition_normalizedMechanismRotations();
    }

    double h_r = 0.7366; //height of robot
    double l_r = 0.6858; //length of robot
    double s_x = l_r / 2; // distance from bumper to base of shooter
    double h_1 = 1.98; //ground to low top
    double h_2 = 2.11; //ground to high top
    double y = 0.459994; // distance from point at end of imaginary line going down from h_1 to point at end of imaginary line going down from h_2 hitting the ground
    double x = 0.472186; // distance from point at end of imaginary line going down from h_1 to robot
    double l_s = 0.3556; //length of shooter
    // EVERYTHING IS IN SI UNITS (M)
    public double calculateMinimalAngleForSpeaker()
    {
        //theta = arctan((h_1 - h_r - l_s * sin(theta))/(x + y + s_x - l_s * cos(theta))
        return 0.12302677222;
    }
    public double calculateMaximalAngleForSpeaker(){
        //theta = arctan((h_2 - h_r - l_s * sin(theta))/(x + s_x - l_s * cos(theta))
        return 0.16472536666;
    }

}

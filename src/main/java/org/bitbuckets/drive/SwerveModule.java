package org.bitbuckets.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import xyz.auriium.mattlib2.hard.IMotorController;
import xyz.auriium.mattlib2.hard.IRotationEncoder;

public class SwerveModule {

    final IMotorController driveMotor;
    final IMotorController steerMotor;
    final IRotationEncoder absoluteEncoder;
    final double sensorPositionCoefficient;

    public SwerveModule(IMotorController driveMotor, IMotorController steerMotor, IRotationEncoder absoluteEncoder, double sensorPositionCoefficient) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.absoluteEncoder = absoluteEncoder;
        this.sensorPositionCoefficient = sensorPositionCoefficient;
    }

    //state
    double referenceAngleRadians = 0;
    int resetIteration = 0;


    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveMotor.linearMechanismPosition_meters(),
                Rotation2d.fromRotations(
                        absoluteEncoder.angularMechanismPosition_wrappedRot()
                )
        );
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveMotor.linearMechanismVelocity_metersPerSecond(),
                Rotation2d.fromRotations(
                        absoluteEncoder.angularMechanismPosition_wrappedRot()
                )
        );
    }

    public double getDriveVelocity() {
        return driveMotor.linearMechanismVelocity_metersPerSecond();
    }

    public double getSteerAngle() {
        return absoluteEncoder.angularMechanismPosition_wrappedRot();
    }

    /**
     * Commands all the motors to immediately stop moving. If this is not called, motors commanded with set will
     * move until eternity
     */
    public void stopAllMotors() {
        driveMotor.setToVoltage(0);
        steerMotor.setToVoltage(0);
    }

    public void set(double driveVoltage, double referenceAngleRadians) {

        //set turn

        referenceAngleRadians %= (2.0 * Math.PI);
        if (referenceAngleRadians < 0.0) {
            referenceAngleRadians += 2.0 * Math.PI;
        }

        double difference = referenceAngleRadians - getSteerAngle();
        // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)
        if (difference >= Math.PI) {
            referenceAngleRadians -= 2.0 * Math.PI;
        } else if (difference < -Math.PI) {
            referenceAngleRadians += 2.0 * Math.PI;
        }
        difference = referenceAngleRadians - getSteerAngle(); // Recalculate difference

        // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
        // movement of the module is less than 90 deg
        if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
            // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
            referenceAngleRadians += Math.PI;
            driveVoltage *= -1.0;
        }

        // Put the target angle back into the range [0, 2pi)
        referenceAngleRadians %= (2.0 * Math.PI);
        if (referenceAngleRadians < 0.0) {
            referenceAngleRadians += 2.0 * Math.PI;
        }
/*

        double currentAngleRadians = steerMotor.getPositionRaw() * sensorPositionCoefficient;

        // Reset the NEO's encoder periodically when the module is not rotating.
        // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
        // end up getting a good reading. If we reset periodically this won't matter anymore.
        if (steerMotor.getVelocityRaw() * (sensorPositionCoefficient * 10) < Math.toRadians(0.5)) {
            if (++resetIteration >= 500) {
                resetIteration = 0;
                double absoluteAngle = absoluteEncoder.getAbsoluteAngle();
                steerMotor.forceOffset(absoluteAngle / sensorPositionCoefficient);
                currentAngleRadians = absoluteAngle;
            }
        } else {
            resetIteration = 0;
        }

        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Falcon's encoder can go above that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

        steerMotor.moveToReference(adjustedReferenceAngleRadians / sensorPositionCoefficient);

        //set drive

        driveMotor.setToVoltage(driveVoltage);

*/

        this.referenceAngleRadians = referenceAngleRadians;
    }

}

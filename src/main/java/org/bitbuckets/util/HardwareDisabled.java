package org.bitbuckets.util;

import org.bitbuckets.disabled.*;
import xyz.auriium.mattlib2.hardware.*;

public class HardwareDisabled {

    public static ILinearVelocityController linearMotor_velocityPID() {
        return new DisabledLinearVelocityController();
    }

    public static ILinearMotor linearMotor_disabled() {
        return new DisabledILinearMotor();
    }

    public static IRotationalMotor rotationalMotor_disabled() {
        return new DisabledIRotationalMotor();
    }

    public static IRotationEncoder rotationEncoder_disabled() {
        return new DisabledIRotationEncoder();
    }

    public static ILinearController linearController_disabled() {
        return new DisabledILinearController();
    }

    public static IRotationalController rotationalController_disabled() {return new DisabledIRotationalController();
    }

    public static IRotationalVelocityController rotationalVelocityController_disabled() {
        return new IRotationalVelocityController() {
            @Override public void controlToRotationalVelocityReferenceArbitrary(double v, double v1) {

            }

            @Override public void setToVoltage(double v) {

            }

            @Override public void stopActuator() {

            }

            @Override public void setToPercent(double v) {

            }

            @Override public double reportCurrentNow_amps() {
                return 0;
            }

            @Override public double reportVoltageNow() {
                return 0;
            }

            @Override public double reportTemperatureNow() {
                return 0;
            }

            @Override public void forceRotationalOffset(double v) {

            }

            @Override public double angularPosition_encoderRotations() {
                return 0;
            }

            @Override public double angularPosition_mechanismRotations() {
                return 0;
            }

            @Override public double angularPosition_normalizedMechanismRotations() {
                return 0;
            }

            @Override public double angularPosition_normalizedEncoderRotations() {
                return 0;
            }

            @Override public double angularVelocity_mechanismRotationsPerSecond() {
                return 0;
            }

            @Override public double angularVelocity_encoderRotationsPerSecond() {
                return 0;
            }
        };
    }

}

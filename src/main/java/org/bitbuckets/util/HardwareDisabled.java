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

}

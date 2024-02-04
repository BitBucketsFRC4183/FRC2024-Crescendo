
package org.bitbuckets.util;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import net.bytebuddy.implementation.bytecode.Throw;
import xyz.auriium.mattlib2.IPeriodicLooped;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.utils.AngleUtil;


public class ThroughBoreEncoder implements IRotationEncoder, IPeriodicLooped {

    Encoder encoder;
    AbsoluteEncoderComponent encoderComponent;


    public ThroughBoreEncoder(Encoder encoder, AbsoluteEncoderComponent encoderComponent) {
        this.encoder = encoder;
        this.encoderComponent = encoderComponent;
        // setDistancePerPulse is 1/pulses per revolution
        // Pulses per revolution is lead of screw/linear resolution
        // Configures the encoder to return a 1 rotation for every x pulses
        // Also changes the units of getRate
        System.out.print(encoder.get());
        double pulsePerSecond = 1/360d;
        encoder.setDistancePerPulse(pulsePerSecond); // TODO check this value please!

        mattRegister();
    }

    @Override
    public void logPeriodic() {
        encoderComponent.logPositionWithOffset(angularPosition_mechanismRotations());
        encoderComponent.logVelocity(angularVelocity_encoderRotationsPerSecond());
    }

    @Override
    public void forceRotationalOffset(double v) {
    }

    @Override
    public double angularPosition_encoderRotations() {
        return encoder.getDistance();
        //throw new UnsupportedOperationException("This encoder is mounted directly on the axis, hence will only read Mechanism rotations and not Encoder Rotations. Please use angularPosition_mechanismRotations() instead!");
    }

    @Override
    public double angularPosition_mechanismRotations() {
        return encoder.getDistance() * encoderComponent.encoderToMechanismCoefficient();
    }

    @Override
    public double angularPosition_normalizedMechanismRotations() {
        return AngleUtil.normalizeRotations(angularPosition_mechanismRotations());
    }

    @Override
    public double angularPosition_normalizedEncoderRotations() {
        return AngleUtil.normalizeRotations(angularPosition_encoderRotations());
        //throw new UnsupportedOperationException("This encoder is mounted directly on the axis, hence will only read Mechanism rotations and not Encoder Rotations. Please use angularPosition_normaliezdMechanismRotations() instead!");
    }

    @Override
    public double angularVelocity_mechanismRotationsPerSecond() {
        return encoder.getRate() * encoderComponent.encoderToMechanismCoefficient();
    }

    @Override
    public double angularVelocity_encoderRotationsPerSecond() {
        //System.out.println("Logging encoder raw: " + encoder.getRaw());
        return encoder.getRate(); //in rotations per minute (double check tho)
        // make sure the switch on the physical through bore encoder is set to A mode (DOES NOT STAND FOR ABSOLUTE ENCODER)
        //throw new UnsupportedOperationException("This encoder is mounted directly on the axis, hence will only read Mechanism rotations and not Encoder Rotations. Please use angularVelocity_mechanismRotationsPerSecond() instead!");
    }
}



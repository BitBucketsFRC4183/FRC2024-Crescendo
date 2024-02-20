package org.bitbuckets.shooter;

import org.bitbuckets.util.AnalogEncoderComponent;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.hardware.IRotationalController;
import xyz.auriium.mattlib2.hardware.IRotationalMotor;

public class ShooterTest {

    @Test
    public void testShooterMathWorkingOk() {

        double deadband = 3;
        double target = 40;
        double actualValue = 43;


        boolean output = PivotSubsystem.isWithinDeadband(deadband, target, actualValue);
        Assertions.assertTrue(output);

    }

    @Test
    public void testHasReachedSpeeds() {

        double speed1 = 4;
        double speed2 = 6;

        var leftMotor = Mockito.mock(IRotationalMotor.class);
        var rightMotor = Mockito.mock(IRotationalMotor.class);
        var rotationalController = Mockito.mock(IRotationalController.class);
        var absoluteEncoder = Mockito.mock(IRotationEncoder.class);
        var shooterComponent = Mockito.mock(ShooterComponent.class);
        var encoderComponent = Mockito.mock(AnalogEncoderComponent.class);

        Mockito.when(leftMotor.angularVelocity_mechanismRotationsPerSecond()).thenReturn(4000d);
        Mockito.when(rightMotor.angularVelocity_mechanismRotationsPerSecond()).thenReturn(5000d);


        //var subsystem = new ShooterSubsystem(leftMotor, rightMotor, rotationalController, absoluteEncoder, shooterComponent, encoderComponent);

        //.assertTrue(subsystem.hasReachedSpeeds(4000d, 5000d));
        //Assertions.assertFalse(subsystem.hasReachedSpeeds(6000d, 6000d));
        //Assertions.assertFalse(subsystem.hasReachedSpeeds(4001d, 5000d));

    }
}

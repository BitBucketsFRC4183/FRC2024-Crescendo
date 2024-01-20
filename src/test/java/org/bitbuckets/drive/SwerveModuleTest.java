package org.bitbuckets.drive;

import edu.wpi.first.wpilibj.AnalogInput;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.hardware.IRotationalMotor;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class SwerveModuleTest {

    @Test
    public void TestLogicPeriodic() {

        IRotationalMotor steerController = Mockito.mock(IRotationalMotor.class);
        Mockito.when(steerController.angularVelocity_mechanismRotationsPerSecond()).thenReturn(20d);

        IRotationEncoder absoluteEncoder = Mockito.mock(IRotationEncoder.class);
        Mockito.when(absoluteEncoder.angularPosition_normalizedMechanismRotations()).thenReturn(10d);
        double absoluteAngularPosition_infiniteMechanismRotations = absoluteEncoder.angularPosition_normalizedMechanismRotations();
        System.out.print(steerController.angularPosition_mechanismRotations());
        steerController.forceRotationalOffset(absoluteAngularPosition_infiniteMechanismRotations);

        assertEquals(0d, steerController.angularPosition_mechanismRotations(), 0);
    }

}

package org.bitbuckets;

import edu.wpi.first.wpilibj.AnalogInput;
import org.bitbuckets.util.AbsoluteEncoderComponent;
import org.bitbuckets.util.ThriftyAbsoluteEncoder;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

import static org.junit.jupiter.api.Assertions.assertEquals;


public class ThriftyAbsoluteEncoderMathTest {

<<<<<<< HEAD

    @Disabled
    @Test
=======
    
    @Disabled
>>>>>>> auto-paths
    public void checkThriftyAngularPosition_encoderRotations()
    {
        AnalogInput fake = Mockito.mock(AnalogInput.class);
        Mockito.when(fake.getVoltage()).thenReturn(-2.4);

        ThriftyAbsoluteEncoder thriftyAbsoluteEncoder = new ThriftyAbsoluteEncoder(fake, Mockito.mock(AbsoluteEncoderComponent.class));
        assertEquals(-0.5, thriftyAbsoluteEncoder.angularPosition_encoderRotations(), 0);
    }
    
<<<<<<< HEAD

    @Disabled
    @Test
=======
    
    @Disabled
>>>>>>> auto-paths
    public void checkThriftyAngularPosition_normalizedEncoderRotations()
    {
        AnalogInput fake = Mockito.mock(AnalogInput.class);
        Mockito.when(fake.getVoltage()).thenReturn(-4.8);

        ThriftyAbsoluteEncoder thriftyAbsoluteEncoder = new ThriftyAbsoluteEncoder(fake, Mockito.mock(AbsoluteEncoderComponent.class));
        assertEquals(0, thriftyAbsoluteEncoder.angularPosition_normalizedEncoderRotations(), 0);
    }

<<<<<<< HEAD

    @Disabled
    @Test
=======
    
    @Disabled
>>>>>>> auto-paths
    public void checkThriftyAngularPosition_mechanismRotations()
    {
        AnalogInput fake = Mockito.mock(AnalogInput.class);
        Mockito.when(fake.getVoltage()).thenReturn(2d);
        AbsoluteEncoderComponent component =  Mockito.mock(AbsoluteEncoderComponent.class);
        Mockito.when(component.encoderToMechanismCoefficient()).thenReturn(3d);

        ThriftyAbsoluteEncoder thriftyAbsoluteEncoder = new ThriftyAbsoluteEncoder(fake,component);

        assertEquals(1.25, thriftyAbsoluteEncoder.angularPosition_mechanismRotations(), 0);
    }

<<<<<<< HEAD

    @Disabled
    @Test
=======
    
    @Disabled
>>>>>>> auto-paths
    public void checkThriftyAngularPosition_NormalizedMechanismRotations()
    {
        AnalogInput fake = Mockito.mock(AnalogInput.class);
        Mockito.when(fake.getVoltage()).thenReturn(-2d);
        AbsoluteEncoderComponent component =  Mockito.mock(AbsoluteEncoderComponent.class);
        Mockito.when(component.encoderToMechanismCoefficient()).thenReturn(3d);

        ThriftyAbsoluteEncoder thriftyAbsoluteEncoder = new ThriftyAbsoluteEncoder(fake, component);
        assertEquals(0.75, thriftyAbsoluteEncoder.angularPosition_normalizedMechanismRotations(), 0.1);
    }
    



}

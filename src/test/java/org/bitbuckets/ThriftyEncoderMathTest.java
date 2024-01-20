package org.bitbuckets;

import edu.wpi.first.wpilibj.AnalogInput;
import org.bitbuckets.util.EncoderComponent;
import org.bitbuckets.util.ThriftyEncoder;
import org.bitbuckets.util.Util;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.Mockito;

import static org.junit.jupiter.api.Assertions.assertEquals;


public class ThriftyEncoderMathTest {

    
    @Test
    public void checkThriftyAngularPosition_encoderRotations()
    {
        AnalogInput fake = Mockito.mock(AnalogInput.class);
        Mockito.when(fake.getVoltage()).thenReturn(-2.4);

        ThriftyEncoder thriftyEncoder = new ThriftyEncoder(fake, Mockito.mock(EncoderComponent.class));
        assertEquals(-0.5, thriftyEncoder.angularPosition_encoderRotations(), 0);
    }
    
    
    @Test
    public void checkThriftyAngularPosition_normalizedEncoderRotations()
    {
        AnalogInput fake = Mockito.mock(AnalogInput.class);
        Mockito.when(fake.getVoltage()).thenReturn(-4.8);


        ThriftyEncoder thriftyEncoder = new ThriftyEncoder(fake, Mockito.mock(EncoderComponent.class));
        assertEquals(0, thriftyEncoder.angularPosition_normalizedEncoderRotations(), 0);
    }

    
    @Test
    public void checkThriftyAngularPosition_mechanismRotations()
    {
        AnalogInput fake = Mockito.mock(AnalogInput.class);
        Mockito.when(fake.getVoltage()).thenReturn(2d);
        EncoderComponent component =  Mockito.mock(EncoderComponent.class);
        Mockito.when(component.getEncoderToMechanismCoefficient()).thenReturn(3d);

        ThriftyEncoder thriftyEncoder = new ThriftyEncoder(fake,component);

        assertEquals(1.25, thriftyEncoder.angularPosition_mechanismRotations(), 0);
    }

    
    @Test
    public void checkThriftyAngularPosition_NormalizedMechanismRotations()
    {
        AnalogInput fake = Mockito.mock(AnalogInput.class);
        Mockito.when(fake.getVoltage()).thenReturn(-2d);
        EncoderComponent component =  Mockito.mock(EncoderComponent.class);
        Mockito.when(component.getEncoderToMechanismCoefficient()).thenReturn(3d);

        ThriftyEncoder thriftyEncoder = new ThriftyEncoder(fake, component);
        assertEquals(0.75, thriftyEncoder.angularPosition_normalizedMechanismRotations(), 0.1);
    }
    



}

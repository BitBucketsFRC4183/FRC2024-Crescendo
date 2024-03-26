package org.bitbuckets;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import edu.wpi.first.wpilibj.util.Color;

public class LedSubsystem {

    final AddressableLED ledStrip;
    final AddressableLEDBuffer buffer;


    public LedSubsystem() {
        this.ledStrip = new AddressableLED(9);
        this.buffer = new AddressableLEDBuffer(60);

        ledStrip.setLength(buffer.getLength());
        ledStrip.setData(buffer);

        ledStrip.start();
        setColor(Color.kRed);
    }

    public void setColor(Color color) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
        ledStrip.setData(buffer);
    }

    public void setColor(Color[] colors) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, colors[i]);
        }
        ledStrip.setData(buffer);
    }


}

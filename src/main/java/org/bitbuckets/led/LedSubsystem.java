package org.bitbuckets.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import xyz.auriium.mattlib2.loop.IMattlibHooked;


public class LedSubsystem implements Subsystem, IMattlibHooked {

    public final NoteManagementSubsystem nms;
    final AddressableLED ledStrip;
    final AddressableLEDBuffer buffer;

    int rainbowFirstPixelHue = 0;
    int PWM_header = 9;
    final boolean lastState;


    public LedSubsystem(NoteManagementSubsystem nms) {
        this.nms = nms;
        this.ledStrip = new AddressableLED(PWM_header);
        this.buffer = new AddressableLEDBuffer(60);
        ledStrip.setLength(buffer.getLength());

        this.lastState = nms.isNoteIn();

        setBufferColor(Color.kAntiqueWhite);
        ledStrip.setData(buffer);

        ledStrip.start();

        register();
        mattRegister();
    }

    @Override
    public void logicPeriodic() {
        if (!DriverStation.isTeleopEnabled()) {
            rainbow();
        } else {
            if (this.lastState != this.nms.isNoteIn()) {
                if (this.nms.isNoteIn()) {
                    setBufferColor(Color.kHoneydew);
                } else setBufferColor(Color.kMagenta);
            }
        }

        ledStrip.setData(buffer);
    }

    private void setBufferColor(Color color) {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
    }

    private void rainbow() {
        // For every pixel
        for (var i = 0; i < buffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
            // Set the value
            buffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        var speed = 3;
        rainbowFirstPixelHue += speed;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }
}


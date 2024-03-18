package org.bitbuckets.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import xyz.auriium.mattlib2.loop.IMattlibHooked;


public class LedSubsytem implements Subsystem, IMattlibHooked {

    public final NoteManagementSubsystem nms;
    final AddressableLED ledStrip;
    final AddressableLEDBuffer buffer;

    final boolean lastState;


    public LedSubsytem(NoteManagementSubsystem nms) {
        this.nms = nms;
        int PWM_header = 1;
        this.ledStrip = new AddressableLED(PWM_header);
        this.buffer = new AddressableLEDBuffer(60);
        ledStrip.setLength(buffer.getLength());

        this.lastState = nms.isNoteIn();
    }


    public void periodic() {
        if (this.lastState == this.nms.isNoteIn()) {
            return;
        } else {
            if (this.nms.isNoteIn()) {
                setColor(Color.kHoneydew);
            } else setColor(Color.kMagenta);
        }
    }

    public void setColor(Color color) {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
    }

    }


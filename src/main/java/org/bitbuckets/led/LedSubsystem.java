package org.bitbuckets.led;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.RobotContainer;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.FlywheelSubsystem;
import org.ojalgo.array.operation.ROT;
import xyz.auriium.mattlib2.auto.ff.RotationFFGenRoutine;
import xyz.auriium.mattlib2.loop.IMattlibHooked;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;


public class LedSubsystem implements Subsystem, IMattlibHooked {

    final NoteManagementSubsystem nms;
    final AddressableLED ledStrip;
    final AddressableLEDBuffer buffer;
    final FlywheelSubsystem flywheel;
    final List<List<Integer>> strips = new ArrayList<>(3);
    int rainbowFirstPixelHue = 0;

    final int PWM_header = 9;
    boolean lastNote;
    ledState currentState;
    ledState lastState;
    double offset = 0;
    public enum ledState {
        IDLE,
        TELEOP
    }

    public LedSubsystem(NoteManagementSubsystem nms, FlywheelSubsystem flywheelSubsystem) {
        this.nms = nms;
        this.ledStrip = new AddressableLED(PWM_header);
        this.buffer = new AddressableLEDBuffer(60);
        this.flywheel = flywheelSubsystem;
        ledStrip.setLength(buffer.getLength());
        setBufferColor(Color.kMagenta);
        ledStrip.setData(buffer);
        ledStrip.start();


        Integer[] intervals = List.of(20, 20, 20).toArray(new Integer[3]);
        var total = 0;
        for (Integer interval : intervals) {
            List<Integer> strip = new ArrayList<>();
            total += interval;

            for (var j = total - interval; j < total; j++) {
                strip.add(j);
            }
            strips.add(strip);
        }
        System.out.println(strips);
        register();
        mattRegister();
    }


    @Override
    public void logicPeriodic() {
        if (!DriverStation.isTeleopEnabled()) { this.currentState = ledState.IDLE; }
        else { this.currentState = ledState.TELEOP; }

        // if idling, rainbow, (else on state switch, and subsequent nms changes update nms buffer)
        if (this.currentState == ledState.IDLE) {rainbowLoop();}
        else {if (this.lastNote != this.nms.isNoteIn() || ((this.lastState != this.currentState))) {nmsIndicator();}}

        lerpGradient(List.of(Color.kAliceBlue, Color.kHotPink, Color.kFloralWhite, Color.kHotPink, Color.kAliceBlue).toArray(new Color[0]));
        ledStrip.setData(buffer);
        this.lastNote = this.nms.isNoteIn();
        this.lastState = this.currentState;
        RobotContainer.LED.log_ledState(this.currentState.toString());


    }

    private void setBufferColor(Color color) {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
    }

    private void rainbowLoop() {
        // For every pixel
        for (var i = 0; i < buffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
            // Set the value
            buffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        var speed = 2;
        rainbowFirstPixelHue += speed;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }

    private void nmsIndicator() {
        if (this.nms.isNoteIn()) {
            setBufferColor(Color.kFirstRed);
        } else setBufferColor(Color.kLimeGreen);
    }

    private void lerpGradient(Color[] colors) {
        // per color, create a gradient
        // i = color
        // j = led #

        // f(t) = {lerp(c1,c2, clamp(t)) u lerp(c2,c3, clamp(t))...}
        List<List<String>> lists = new ArrayList<>();

        for (var i = 0; i < colors.length; i++) {
            lists.add(new ArrayList<>());
            for (var j = 0; j < buffer.getLength() / colors.length; j++) {

                Color currentColor = colors[i];
                Color nextColor = colors[(i + 1) % colors.length];


                double r = MathUtil.interpolate(currentColor.red, nextColor.red, ((double) j / (buffer.getLength()))) * 255;
                double g = MathUtil.interpolate(currentColor.green, nextColor.green, ((double) j / (buffer.getLength()))) * 255;
                double b = MathUtil.interpolate(currentColor.blue, nextColor.blue, ((double) j / (buffer.getLength()))) * 255;

                int ledNumber = (j + (buffer.getLength() / colors.length * i) + (int) offset) % buffer.getLength();
                buffer.setRGB(ledNumber, (int) r, (int) g, (int) b);
                lists.get(i).add(buffer.getLED(ledNumber).toString());
                // Calculate the hue - hue is easier for rainbows because the color
                // shape is a circle so only one value needs to precess
            }
        }
        // movement for offset
        offset++;
        if (offset == buffer.getLength()) {
            offset = 0;
        }
    }

    private void dynamicShooterSpeedsColoredAndScaledIndicatorLightThreeBarSeperated() {
        double targetSpeed = RobotContainer.COMMANDS.ramFireSpeed_mechanismRotationsPerSecond();
        double lRatio = MathUtil.clamp(Math.abs(flywheel.velocityEncoderLeft.angularVelocity_mechanismRotationsPerSecond()) / targetSpeed, 0, 1);
        double rRatio = MathUtil.clamp(Math.abs(flywheel.velocityEncoderRight.angularVelocity_mechanismRotationsPerSecond()) / targetSpeed, 0, 1);

        setBufferColor(Color.kWhiteSmoke);
        List<Integer> leftStrip =  strips.get(0);
        for (var i = 0; i < leftStrip.size(); i++) {
            if ((double) i / leftStrip.size() <= lRatio) {
                buffer.setLED(leftStrip.get(i), Color.kPurple);
            }
        }

        List<Integer> rightStrip =  strips.get(2);
        for (var i = 0; i < rightStrip.size(); i++) {
            if ((double) i / rightStrip.size() <= rRatio) {
                buffer.setLED(rightStrip.get(i), Color.kPurple);
            }
        }

        List<Integer> centerStrip = strips.get(1);
        List<List<Integer>> centersInverted = List.of(Arrays.copyOfRange(centerStrip,0, centerStrip.size()/2))
    }
}





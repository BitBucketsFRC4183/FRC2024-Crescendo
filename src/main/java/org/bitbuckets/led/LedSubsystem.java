package org.bitbuckets.led;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.RobotContainer;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.FlywheelSubsystem;
import org.ojalgo.array.operation.ROT;
import org.opencv.core.Mat;
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
    final GroundIntakeSubsystem intake;
    final List<List<Integer>> strips = new ArrayList<>(3);
    final List<Integer> allStrip = new ArrayList<>();
    int rainbowFirstPixelHue = 0;

    final int PWM_header = 9;
    ledState currentState;
    ledState lastState;
    double offset = 0;
    double lRatio;
    double rRatio;
    double avgRatio;

    public enum ledState {
        IDLE,
        TELEOP,
        SHOOTING,
        INTAKE,
        OUTTAKE
    }

    public LedSubsystem(NoteManagementSubsystem nms, FlywheelSubsystem flywheelSubsystem, GroundIntakeSubsystem groundIntakeSubsystem) {
        this.nms = nms;
        this.ledStrip = new AddressableLED(PWM_header);
        this.buffer = new AddressableLEDBuffer(60);
        this.flywheel = flywheelSubsystem;
        this.intake = groundIntakeSubsystem;


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
                allStrip.add(j);
            }
            strips.add(strip);
        }


        register();
        mattRegister();
    }


    @Override
    public void logicPeriodic() {
        double targetSpeed = RobotContainer.COMMANDS.ramFireSpeed_mechanismRotationsPerSecond();
        lRatio = MathUtil.clamp(Math.abs(flywheel.velocityEncoderLeft.angularVelocity_mechanismRotationsPerSecond()) / targetSpeed, 0, 1);
        rRatio = MathUtil.clamp(Math.abs(flywheel.velocityEncoderRight.angularVelocity_mechanismRotationsPerSecond()) / targetSpeed, 0, 1);
        avgRatio = MathUtil.clamp((Math.abs(flywheel.velocityEncoderRight.angularVelocity_mechanismRotationsPerSecond()) + Math.abs(flywheel.velocityEncoderLeft.angularVelocity_mechanismRotationsPerSecond())) / 2 / targetSpeed, 0, 1);

        if (avgRatio > 0.02) { this.currentState = ledState.SHOOTING; }
        else if (intake.bottomMotor.reportVoltageNow() > 1 || intake.topMotor.reportVoltageNow() > 1) {this.currentState = ledState.INTAKE;}
        else if (intake.bottomMotor.reportVoltageNow() < -1 || intake.topMotor.reportVoltageNow() < -1) {this.currentState = ledState.OUTTAKE;}
        else if (!DriverStation.isTeleopEnabled()) { this.currentState = ledState.IDLE; }
        else { this.currentState = ledState.TELEOP; }


        /// this.currentState = ledState.INTAKE;
        if (this.currentState == ledState.IDLE) {
            rainbowLoop();
        }
        else if (this.currentState == ledState.SHOOTING) {dynamicShooterSpeedsColoredAndScaledIndicatorLightThreeBarSeperated();}
        else if (this.currentState == ledState.INTAKE) {intakeRunning(false);}
        else if (this.currentState == ledState.OUTTAKE) {intakeRunning(true);}
        else {nmsIndicator();}

        
        // lerpGradient(List.of(Color.kLightBlue, Color.kPink, Color.kWhite, Color.kPink, Color.kLightBlue).toArray(new Color[0]));
        // dynamicShooterSpeedsColoredAndScaledIndicatorLightThreeBarSeperated();
        // intakeRunning(false);
        ledStrip.setData(buffer);
        this.lastState = this.currentState;
        RobotContainer.LED.log_ledState(this.currentState.toString());
    }

    @Override
    public void logPeriodic() {
        for (List<Integer> strip : strips) {
            List<String> colors = new ArrayList<>();
            for (Integer number : strip) {

                colors.add(buffer.getLED(number).toString());
            }
            System.out.println(colors);
        }
        System.out.println(strips);

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

        var speed = 1;
        rainbowFirstPixelHue += speed;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }

    private void nmsIndicator() {
        if (this.nms.isNoteIn()) {
            setBufferColor(Color.kRed);
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

                float sigmoidConstant = 7;
                double r = MathUtil.interpolate(currentColor.red, nextColor.red, lerpSigmoid((float) j / (buffer.getLength()), sigmoidConstant)) * 255;
                double g = MathUtil.interpolate(currentColor.green, nextColor.green, lerpSigmoid((float) j / (buffer.getLength()), sigmoidConstant)) * 255;
                double b = MathUtil.interpolate(currentColor.blue, nextColor.blue, lerpSigmoid((float) j / (buffer.getLength()), sigmoidConstant)) * 255;

                int ledNumber = (j + (buffer.getLength() / colors.length * i) + (int) offset) % buffer.getLength();
                buffer.setRGB(ledNumber, (int) r, (int) g, (int) b);
                lists.get(i).add(buffer.getLED(ledNumber).toString());
                // Calculate the hue - hue is easier for rainbows because the color
                // shape is a circle so only one value needs to precess
            }
        }
        // movement for offset
        offset++;
        if (offset % buffer.getLength() == 0) {
            offset = 0;
        }
    }

    private void dynamicShooterSpeedsColoredAndScaledIndicatorLightThreeBarSeperated() {
        // funee monkey
        setBufferColor(Color.kWhiteSmoke);
        List<Integer> leftStrip =  strips.get(0);
        for (var i = 0; i < leftStrip.size(); i++) {
            if ((double) i / leftStrip.size() <= lRatio) {
                buffer.setLED(leftStrip.get(i), Color.kPurple);
            }
        }

        List<Integer> rightStrip =  new ArrayList<>(strips.get(2));
        Collections.reverse(rightStrip);
        System.out.println(rightStrip);
        for (var i = 0; i < rightStrip.size(); i++) {
            if ((double) i / rightStrip.size() <= rRatio) {
                buffer.setLED(rightStrip.get(i), Color.kPurple);
            }
        }

        List<Integer> centerStrip = strips.get(1);
        List<List<Integer>> centers = List.of(new ArrayList<>(), new ArrayList<>());

        for (var j = 0; j < centerStrip.size()/2; j++) {
            centers.get(0).add(centerStrip.get(j));
        }
        Collections.reverse(centers.get(0));

        for (var j = centerStrip.size()/2; j < centerStrip.size(); j++) {
            centers.get(1).add(centerStrip.get(j));
        }

        for (List<Integer> strip : centers) {
            for (var i = 0; i < strip.size(); i++) {
                if ((double) i / strip.size() <= avgRatio) {
                    buffer.setLED(strip.get(i), Color.kPurple);
                }
            }
        }
    }

    private void intakeRunning(boolean inverted) {
        List<List<Integer>> halves = List.of(new ArrayList<>(), new ArrayList<>());

        for (var j = 0; j < allStrip.size()/2; j++) {
            halves.get(0).add(allStrip.get(j));
        }
        Collections.reverse(halves.get(0));

        for (var j = allStrip.size()/2; j < allStrip.size(); j++) {
            halves.get(1).add(allStrip.get(j));
        }

        var chunkSize = 3;
        Color color;
        if (this.nms.isNoteIn()) { color = Color.kLimeGreen; } else { color = Color.kRed; }

        setBufferColor(Color.kBlack);
        if (inverted) {
            Collections.reverse(halves.get(0));
            Collections.reverse(halves.get(1));
        }

        for (var i = 0; i < halves.get(0).size(); i++) {
            if (Math.ceil((double) i / chunkSize) % 2 == 1) {
                buffer.setLED((int) (halves.get(0).get(i) + offset) % allStrip.size(), color);
                buffer.setLED((int) (halves.get(1).get(i) + offset) % allStrip.size(), color);
            }
        }

        offset++;
        if (offset % chunkSize == 0) {
            offset = 0;
        }
    }

    // sigmoid the T value of lerp (domain is [0, 1], range is [0,1])
    private static float lerpSigmoid(float x, float k) {
        return  (1 / (1 + (float) Math.pow(x/(1-x), -k)));
    }
}





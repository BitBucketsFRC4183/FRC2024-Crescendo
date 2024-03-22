package org.bitbuckets.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.LedSubsystem;
import org.bitbuckets.shooter.FlywheelSubsystem;
import yuukonfig.core.ArrayUtil;

public class SetFlywheelLEDCommand extends Command {

    final LedSubsystem ledSubsystem;
    final FlywheelSubsystem flywheelSubsystem;

    public SetFlywheelLEDCommand(LedSubsystem ledSubsystem, FlywheelSubsystem flywheelSubsystem) {
        this.ledSubsystem = ledSubsystem;
        this.flywheelSubsystem = flywheelSubsystem;
    }


    @Override
    public void execute() {
        Color[] leftArr = split(flywheelSubsystem.getLeftPercentage());
        Color[] rightArr = split(flywheelSubsystem.getRightPercentage());
        Color[] avgArr = split(flywheelSubsystem.getAveragePercentage());
        Color[] sum = ArrayUtil.combine(ArrayUtil.combine(leftArr, rightArr), avgArr);

        ledSubsystem.setColor(sum);
    }

    static Color[] split(double percentage) {
        Color[] arr = new Color[20];
        int toColor = (int)( percentage * 20);
        int toDelta = toColor * 1;

        for (int i = 0; i < toColor; i++) {
            arr[i] = Color.kAliceBlue;
        }
        for (int i = toDelta; i < 20; i++) {
            arr[i] = Color.kWhite;
        }

        return arr;
    }

    @Override
    public void end(boolean interrupted) {
        ledSubsystem.setColor(Color.kRed);
    }
}

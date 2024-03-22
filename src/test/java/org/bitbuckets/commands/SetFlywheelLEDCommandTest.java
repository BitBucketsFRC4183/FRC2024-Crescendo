package org.bitbuckets.commands;

import edu.wpi.first.wpilibj.util.Color;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import yuukonfig.core.ArrayUtil;

import static org.junit.jupiter.api.Assertions.*;

class SetFlywheelLEDCommandTest {

    @Test
    void splitTest() {

        Color[] allBlue = SetFlywheelLEDCommand.split(1);

        for (int i = 0; i < 20; i++) {
            Assertions.assertEquals(Color.kAliceBlue, allBlue[i]);
        }

        Color[] allWhite = SetFlywheelLEDCommand.split(0);

        for (int i = 0; i < 20; i++) {
            Assertions.assertEquals(Color.kWhite, allWhite[i]);
        }

        Color[] halfAndHalf = SetFlywheelLEDCommand.split(0.5);

        for (int i = 0; i < 10; i++) {
            Assertions.assertEquals(Color.kAliceBlue, halfAndHalf[i]);
        }
        for (int i = 10; i < 20; i++) {
            Assertions.assertEquals(Color.kWhite, halfAndHalf[i]);
        }
    }

    @Test
    void shouldSumBeLessThan60() {

        Color[] leftArr = SetFlywheelLEDCommand.split(1d);
        Color[] rightArr = SetFlywheelLEDCommand.split(1d);
        Color[] avgArr = SetFlywheelLEDCommand.split(1d);
        Color[] sum = ArrayUtil.combine(ArrayUtil.combine(leftArr, rightArr), avgArr);

        Assertions.assertEquals(60, sum.length);

        Color[] leftArr1 = SetFlywheelLEDCommand.split(0.2);
        Color[] rightArr1 = SetFlywheelLEDCommand.split(0.4);
        Color[] avgArr1 = SetFlywheelLEDCommand.split(0.9999999992);
        Color[] sum1 = ArrayUtil.combine(ArrayUtil.combine(leftArr1, rightArr1), avgArr1);

        Assertions.assertEquals(60, sum1.length);

    }
}
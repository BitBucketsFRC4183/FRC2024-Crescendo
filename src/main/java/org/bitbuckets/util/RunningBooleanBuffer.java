package org.bitbuckets.util;

import java.util.Arrays;

public class RunningBooleanBuffer {

    final boolean[] buffer;

    public RunningBooleanBuffer(int size) {
        this.buffer = new boolean[size];

        for (int i = 0; i < size; i++) {
            buffer[i] = false;
        }
    }

    public void flag(boolean flag) {
        //push every index back

        boolean[] newBuffer = new boolean[buffer.length];

        newBuffer[0] = flag;
        //ignore the last element
        System.arraycopy(buffer, 0, newBuffer, 1, buffer.length - 1);
        System.arraycopy(newBuffer, 0, buffer, 0, buffer.length);
    }

    public boolean averageBooleans(int quantityThatMatch) {
        double count = 0;

        for (boolean d : buffer) {
            if (d) count++;
        }

        return count > quantityThatMatch;
    }

    public void clear() {
        Arrays.fill(buffer, false);
    }


}

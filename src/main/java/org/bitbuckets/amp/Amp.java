package org.bitbuckets.amp;

public interface Amp {
    void retract();  //move back to intial position
    void moveToAmp(); //continuosly called


    boolean hasReachedAmp();
}

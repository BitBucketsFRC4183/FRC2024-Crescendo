package org.bitbuckets.amp;

public interface Amp {
    void retract();  //move back to intial position
    void moveToAmp(); //continuosly called

    void doNothing();


    boolean hasReachedAmp();

    boolean hasReachedInitial();

}

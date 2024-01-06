package org.bitbuckets;

import org.bitbuckets.util.Util;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import xyz.auriium.mattlib2.log.ProcessPath;

public class UtilTest {

    @Test
    public void renamerShouldWork() {

        String myTestString = "hi/game";

        String out = Util.RENAMER.apply(myTestString, 0);

        Assertions.assertEquals("hi/fl/game/", out);

    }

}

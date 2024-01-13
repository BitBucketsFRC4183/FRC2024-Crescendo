package org.bitbuckets;

import xyz.auriium.mattlib2.Mattlib;

public class MattlibPrecompileTest {

    public void testMattlibPrecompileShouldWork() {
        Mattlib.COMPILE_CHECKER.throwAllDetectedProblems();
    }

}

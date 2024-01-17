package org.bitbuckets;

import com.choreo.lib.Choreo;
import xyz.auriium.mattlib2.Mattlib;

public class MattlibPrecompileTest {

    public void testMattlibPrecompileShouldWork() {
        Mattlib.COMPILE_CHECKER.throwAllDetectedProblems();
    }

}

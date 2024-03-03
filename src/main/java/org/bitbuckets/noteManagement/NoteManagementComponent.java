package org.bitbuckets.noteManagement;

import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;
import xyz.auriium.mattlib2.log.annote.Essential;
import xyz.auriium.mattlib2.log.annote.Log;

public interface NoteManagementComponent extends INetworkedComponent {
    @Conf("channel") int channel();
    @Conf("deadband") double deadband_mechanismRotations();

    @Essential @Log("noteInTheHole") void reportDIOOutput(boolean output);
}

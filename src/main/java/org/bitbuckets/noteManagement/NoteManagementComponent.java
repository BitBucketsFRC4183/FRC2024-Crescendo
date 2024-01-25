package org.bitbuckets.noteManagement;

import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;

public interface NoteManagementComponent extends INetworkedComponent {
    @Conf("channel") int channel();
    @Conf("deadband") double deadband_mechanismRotations();
    @Conf("dio_channel") int dio();
}

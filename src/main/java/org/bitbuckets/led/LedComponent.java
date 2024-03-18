package org.bitbuckets.led;

import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;

public interface LedComponent extends INetworkedComponent {
    @Conf("channel") int channel();
}

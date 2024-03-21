package org.bitbuckets.led;

import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;
import xyz.auriium.mattlib2.log.annote.Essential;
import xyz.auriium.mattlib2.log.annote.Log;

public interface LedComponent extends INetworkedComponent {
    @Log("current_mode") void log_current_mode(String hi);
}

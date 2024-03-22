package org.bitbuckets.led;

import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;
import xyz.auriium.mattlib2.log.annote.Essential;
import xyz.auriium.mattlib2.log.annote.Log;

public interface LedComponent extends INetworkedComponent {
    @Log("ledState") void log_ledState(String hi);
}

package org.bitbuckets.drive;

import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;

public interface FFComponent extends INetworkedComponent {

    @Conf("ff_ks") double ff_ks();
    @Conf("ff_kv") double ff_kv();

}

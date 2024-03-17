package org.bitbuckets.drive;

import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;

public interface FFComponent extends INetworkedComponent {

    @Conf("ks") double ff_ks();
    @Conf("kv") double ff_kv();
    @Conf("ka") double ff_ka();

}

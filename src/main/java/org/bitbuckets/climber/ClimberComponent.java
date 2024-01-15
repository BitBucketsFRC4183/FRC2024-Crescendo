package org.bitbuckets.climber;

import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;

public interface ClimberComponent extends INetworkedComponent {
    @Conf("ks") double ff_ks(); // TODO
    @Conf("kv") double ff_kv(); // TODO
    @Conf("ka") double ff_ka(); // TODO

    @Conf("dampner") double dampner(); // TODO



}

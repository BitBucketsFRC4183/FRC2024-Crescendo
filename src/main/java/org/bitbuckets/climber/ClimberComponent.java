package org.bitbuckets.climber;

import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;

public interface ClimberComponent extends INetworkedComponent {
    @Conf("ff_ks") double ff_ks(); // TODO
    @Conf("ff_kv") double ff_kv(); // TODO

}

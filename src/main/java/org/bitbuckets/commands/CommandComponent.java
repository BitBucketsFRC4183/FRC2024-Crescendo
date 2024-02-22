package org.bitbuckets.commands;

import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;

public interface CommandComponent extends INetworkedComponent {

    @Conf("ram_fire_speed") double ramFireSpeed_mechanismRotationsPerSecond();
    @Conf("skip_intermediates") boolean skipIntermediates();
}

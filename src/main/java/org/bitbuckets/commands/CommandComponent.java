package org.bitbuckets.commands;

import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;

public interface CommandComponent extends INetworkedComponent {

    @Conf("groupDeadline_seconds") double groupDeadline_seconds();
    @Conf("ramFire_speed") double ramFireSpeed_mechanismRotationsPerSecond();
    @Conf("groundIntake_nominalVoltage") double groundIntake_voltage();
    @Conf("noteManagement_nominalVoltage") double noteManagement_voltage();

    @Conf("skip_intermediates") boolean skipIntermediates();
}

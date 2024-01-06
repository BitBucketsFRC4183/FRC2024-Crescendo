package org.bitbuckets.drive;

import edu.wpi.first.math.geometry.Translation2d;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annotation.Conf;

public interface SwerveComponent extends INetworkedComponent {

    @Conf
    double ff_ks();

    @Conf
    double ff_kv();

    @Conf
    double ff_ka();

    @Conf
    Translation2d[] translations();

}

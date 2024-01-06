package org.bitbuckets.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annotation.Conf;
import xyz.auriium.mattlib2.log.annotation.Log;
import yuukonfig.core.annotate.Key;

public interface SwerveComponent extends INetworkedComponent {

    @Conf
    double ff_ks();

    @Conf
    double ff_kv();

    @Conf
    double ff_ka();

    @Conf
    Translation2d[] translations();

    @Log
    @Key("swervePositions")
    void logSwervePositions(SwerveModulePosition[] positions);

    @Log
    @Key("swerveStates")
    void logSwerveStates(SwerveModuleState[] states);

}

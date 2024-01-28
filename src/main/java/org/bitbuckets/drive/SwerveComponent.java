package org.bitbuckets.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import xyz.auriium.mattlib2.log.annote.Conf;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Log;

public interface SwerveComponent extends INetworkedComponent {

    @Conf("pigeonId") int pigeonCanId();
    @Conf("ff_ks") double ff_ks();
    @Conf("ff_kv") double ff_kv();
    @Conf("ff_ka") double ff_ka();

    @Conf("halfWidth") double halfWidth_meters();
    @Conf("halfBase") double halfBase_meters();

    @Conf("magnitudeSlew") double magnitudeSlew();
    @Conf("angleSlew") double rotationalSlew();

    //@Log("positions") void logSwervePositions(SwerveModulePosition[] positions);
    @Log("states") void logSwerveStates(SwerveModuleState[] states);
    @Log("desiredStates") void logDesiredStates(SwerveModuleState[] desiredStates);
    @Log("pose2") void logPosition(Pose2d pose2d);

}

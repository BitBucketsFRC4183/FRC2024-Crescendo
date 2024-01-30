package org.bitbuckets.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import xyz.auriium.mattlib2.log.annote.Conf;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Log;

public interface SwerveComponent extends INetworkedComponent {

    @Conf("pigeonId") int pigeonCanId();
    @Conf("ff_ks") double ff_ks();
    @Conf("ff_kv") double ff_kv();
    @Conf("ff_ka") double ff_ka();

    @Conf("magnitudeFWLim") double magnitudeFwLimit();

    @Conf("fr_offset") Translation2d fr_offset();
    @Conf("fl_offset") Translation2d fl_offset();
    @Conf("br_offset") Translation2d br_offset();
    @Conf("bl_offset") Translation2d bl_offset();


    //@Log("positions") void logSwervePositions(SwerveModulePosition[] positions);
    @Log("states") void logSwerveStates(SwerveModuleState[] states);
    @Log("desiredStates") void logDesiredStates(SwerveModuleState[] desiredStates);
    @Log("pose2") void logPosition(Pose2d pose2d);

}

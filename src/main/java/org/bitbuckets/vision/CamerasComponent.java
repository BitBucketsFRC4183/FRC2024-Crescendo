package org.bitbuckets.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;

public interface CamerasComponent extends INetworkedComponent {

    @Conf("camera1Name") String camera1Name();

    //offset from robot center of mass/origin and rotation
    @Conf("camera2Name") String camera2Name();

    @Conf("camera1TranslationOffset") Translation3d camera1TranslationOffset();
    @Conf("camera1RotationOffset") Rotation3d camera1RotationOffset();
    @Conf("camera2TranslationOffset") Translation3d camera2TranslationOffset();
    @Conf("camera2RotationOffset") Rotation3d camera2RotationOffset();
}

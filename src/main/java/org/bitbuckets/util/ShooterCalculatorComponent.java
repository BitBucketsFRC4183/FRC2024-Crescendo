package org.bitbuckets.util;

import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;

public interface ShooterCalculatorComponent extends INetworkedComponent{

    @Conf("drag_coefficient") double drag_coefficient();
    @Conf("magnus_coefficient") double magnus_coefficient();
    @Conf("speaker_height") double speaker_height();
    @Conf("speaker_distance") double speaker_distance();
    @Conf("velocity") double velocity();
    @Conf("angle") double angle();



}

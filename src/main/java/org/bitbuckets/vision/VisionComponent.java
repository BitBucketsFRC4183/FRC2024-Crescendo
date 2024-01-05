package org.bitbuckets.vision;

import xyz.auriium.mattlib2.log.annotation.Conf;
import xyz.auriium.mattlib2.log.annotation.Log;
import xyz.auriium.mattlib2.log.components.ILogComponent;

public interface VisionComponent extends ILogComponent {

    @Log
    void x_position(double data);

    @Conf
    int id();

}

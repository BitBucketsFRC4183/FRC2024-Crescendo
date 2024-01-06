package org.bitbuckets.vision;


import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annotation.Conf;
import xyz.auriium.mattlib2.log.annotation.Log;

public interface VisionComponent extends INetworkedComponent {

    @Log
    void x_position(double data);

    @Conf
    int id();

}

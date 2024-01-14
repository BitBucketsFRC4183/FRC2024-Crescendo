package org.bitbuckets.vision;


import xyz.auriium.mattlib2.log.annote.Conf;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Log;

public interface VisionComponent extends INetworkedComponent {

    @Log("x_pos") void x_position(double data);
    @Conf("id") int id();

}

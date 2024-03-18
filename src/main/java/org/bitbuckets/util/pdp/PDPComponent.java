package org.bitbuckets.util.pdp;

import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Log;

public interface PDPComponent extends INetworkedComponent {

    @Log("totalCurrentDraw_amps") void reportCurrentDraw(double currentDraw);
    @Log("currentDrawPerChannel_ampsVector") void reportCurrentPerChannel(double[] current);

}

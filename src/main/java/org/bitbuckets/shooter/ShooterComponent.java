package org.bitbuckets.shooter;

import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;
import xyz.auriium.mattlib2.log.annote.Log;

public interface ShooterComponent extends INetworkedComponent {

   //@Log("voltage") double voltage(double voltage);

   @Conf("ff_ks") double ks();
   @Conf("ff_kv") double kv();


   @Conf("channel") int channel();
   @Conf("deadband") double deadband_mechanismRotations();
   @Conf("dio_channel") int dio();


}

package org.bitbuckets.shooter;

import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;

public interface ShooterComponent extends INetworkedComponent {

   //@Log("voltage") double voltage(double voltage);

   @Conf("ff_ks") double ks();
   @Conf("ff_kv") double kv();


   @Conf("pivotChannel_dio") int pivotChannel_dio();
   @Conf("velocityChannelA_dio") int velocityChannelA_dio();
   @Conf("velocityChannelB_dio") int velocityChannelB_dio();
   @Conf("deadband") double deadband_mechanismRotations();



}

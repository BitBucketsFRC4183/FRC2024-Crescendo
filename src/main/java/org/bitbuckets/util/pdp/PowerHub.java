package org.bitbuckets.util.pdp;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.simulation.PDPSim;
import xyz.auriium.mattlib2.loop.IMattlibHooked;

public class PowerHub implements IMattlibHooked {

    final PowerDistribution powerDistribution;
    final PDPComponent component;
    final double[] channelCurrents;

    public PowerHub(PowerDistribution powerDistribution, PDPComponent component) {
        this.powerDistribution = powerDistribution;
        this.component = component;
        this.channelCurrents = new double[powerDistribution.getNumChannels()];
    }

    @Override public void logPeriodic() {
        component.reportCurrentDraw(powerDistribution.getTotalCurrent());

        for (int i = 0; i < channelCurrents.length; i++) {
            channelCurrents[i] = powerDistribution.getCurrent(i);
        }

        component.reportCurrentPerChannel(channelCurrents);
    }
}

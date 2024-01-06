package org.bitbuckets;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * WRT blue as 0,0
 */
public class FieldConstants {

    static final double FIELD_LENGTH_METERS = 16.54;
    static final double FIELD_WIDTH_METERS = 8.02;

    static final double MIDWIDTH_TO_STEREO_METERS = 1.45;

    static final Translation2d BOTTOM_LEFT = new Translation2d(0,0);
    static final Translation2d TOP_LEFT = new Translation2d(FIELD_WIDTH_METERS, 0);

    Translation2d BLUE_STEREO = new Translation2d((FIELD_WIDTH_METERS / 2d) + MIDWIDTH_TO_STEREO_METERS, 0);
    Translation2d RED_STEREO = new Translation2d((FIELD_WIDTH_METERS / 2d) + MIDWIDTH_TO_STEREO_METERS, FIELD_LENGTH_METERS);


}

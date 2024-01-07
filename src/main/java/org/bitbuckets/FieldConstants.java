package org.bitbuckets;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * WRT blue as 0,0
 */
public class FieldConstants {

    static final double FIELD_X_METERS = 16.54;
    static final double FIELD_Y_METERS = 8.02;

    static final double Y_MIDPOINT_TO_STEREO_METERS = 1.45;
    static final double X_WALL_TO_AMP_METERS = 1.93;

    static final double STAGE_TRIANGLE_BASE_METERS = 3.115;
    static final double STAGE_TRIANGLE_HEIGHT_METERS = 2.697;
    static final double ALLIANCE_WALL_TO_STAGE_CENTROID_METERS = 3.073 + STAGE_TRIANGLE_HEIGHT_METERS * 2.0 / 3.0;

    static final double ALLIANCE_WALL_TO_WING_NOTES_METERS = 2.896;
    static final double WING_NOTES_SEPERATION_METERS = 1.448;
    static final double CENTER_NOTES_SEPERATION_METERS = 1.676;

    static final Translation2d BOTTOM_LEFT = new Translation2d(0,0);
    static final Translation2d TOP_LEFT = new Translation2d(0, FIELD_Y_METERS);

    static final Translation2d BLUE_STEREO = new Translation2d(0, (FIELD_Y_METERS / 2d) + Y_MIDPOINT_TO_STEREO_METERS);
    static final Translation2d RED_STEREO = new Translation2d( FIELD_X_METERS, (FIELD_Y_METERS / 2d) + Y_MIDPOINT_TO_STEREO_METERS);

    static final Translation2d BLUE_AMP = new Translation2d(X_WALL_TO_AMP_METERS, FIELD_Y_METERS);
    static final Translation2d RED_AMP = new Translation2d(FIELD_X_METERS - X_WALL_TO_AMP_METERS, FIELD_Y_METERS);

    Translation2d BLUE_STAGE_CENTER = new Translation2d(ALLIANCE_WALL_TO_STAGE_CENTROID_METERS, (FIELD_Y_METERS / 2d));
    Translation2d RED_STAGE_CENTER = new Translation2d((FIELD_X_METERS - ALLIANCE_WALL_TO_STAGE_CENTROID_METERS), (FIELD_Y_METERS / 2d));

    Translation2d BLUE_STAGE_LEFT = new Translation2d((ALLIANCE_WALL_TO_STAGE_CENTROID_METERS - STAGE_TRIANGLE_HEIGHT_METERS * 2/3), (FIELD_Y_METERS / 2d));
    Translation2d BLUE_STAGE_TOP = new Translation2d((ALLIANCE_WALL_TO_STAGE_CENTROID_METERS + STAGE_TRIANGLE_HEIGHT_METERS * 1/3), (FIELD_Y_METERS / 2d + STAGE_TRIANGLE_BASE_METERS / 2d));
    Translation2d BLUE_STAGE_BOTTOM = new Translation2d((ALLIANCE_WALL_TO_STAGE_CENTROID_METERS + STAGE_TRIANGLE_HEIGHT_METERS * 1/3), (FIELD_Y_METERS / 2d - STAGE_TRIANGLE_BASE_METERS / 2d));

    Translation2d RED_STAGE_RIGHT = new Translation2d((FIELD_X_METERS - ALLIANCE_WALL_TO_STAGE_CENTROID_METERS + STAGE_TRIANGLE_HEIGHT_METERS * 2/3), (FIELD_Y_METERS / 2d));
    Translation2d RED_STAGE_TOP = new Translation2d((FIELD_X_METERS - ALLIANCE_WALL_TO_STAGE_CENTROID_METERS - STAGE_TRIANGLE_HEIGHT_METERS * 1/3), (FIELD_Y_METERS / 2d + STAGE_TRIANGLE_BASE_METERS / 2d));
    Translation2d RED_STAGE_BOTTOM = new Translation2d((FIELD_X_METERS - ALLIANCE_WALL_TO_STAGE_CENTROID_METERS - STAGE_TRIANGLE_HEIGHT_METERS * 1/3), (FIELD_Y_METERS / 2d - STAGE_TRIANGLE_BASE_METERS / 2d));

    Translation2d BLUE_WING_NOTE_BOTTOM = new Translation2d(ALLIANCE_WALL_TO_WING_NOTES_METERS, FIELD_Y_METERS / 2d);
    Translation2d BLUE_WING_NOTE_MID = new Translation2d(ALLIANCE_WALL_TO_WING_NOTES_METERS,    FIELD_Y_METERS / 2d + WING_NOTES_SEPERATION_METERS);
    Translation2d BLUE_WING_NOTE_TOP = new Translation2d(ALLIANCE_WALL_TO_WING_NOTES_METERS, FIELD_Y_METERS / 2d + WING_NOTES_SEPERATION_METERS * 2);

    Translation2d RED_WING_NOTE_BOTTOM = new Translation2d(FIELD_X_METERS - ALLIANCE_WALL_TO_WING_NOTES_METERS, FIELD_Y_METERS / 2d);
    Translation2d RED_WING_NOTE_MID = new Translation2d(FIELD_X_METERS - ALLIANCE_WALL_TO_WING_NOTES_METERS, FIELD_Y_METERS / 2d + WING_NOTES_SEPERATION_METERS);
    Translation2d RED_WING_NOTE_TOP = new Translation2d(FIELD_X_METERS - ALLIANCE_WALL_TO_WING_NOTES_METERS, FIELD_Y_METERS / 2d + WING_NOTES_SEPERATION_METERS * 2);

    Translation2d CENTER_NOTE_BOTTOM = new Translation2d(FIELD_X_METERS / 2d, FIELD_Y_METERS / 2d - CENTER_NOTES_SEPERATION_METERS * 2);
    Translation2d CENTER_NOTE_MID_BOTTOM = new Translation2d(FIELD_X_METERS / 2d, FIELD_Y_METERS / 2d - CENTER_NOTES_SEPERATION_METERS);
    Translation2d CENTER_NOTE_MID = new Translation2d(FIELD_X_METERS / 2d, FIELD_Y_METERS / 2d);
    Translation2d CENTER_NOTE_MID_TOP = new Translation2d(FIELD_X_METERS / 2d, FIELD_Y_METERS / 2d + CENTER_NOTES_SEPERATION_METERS);
    Translation2d CENTER_NOTE_TOP = new Translation2d(FIELD_X_METERS / 2d, FIELD_Y_METERS / 2d + CENTER_NOTES_SEPERATION_METERS * 2);

}

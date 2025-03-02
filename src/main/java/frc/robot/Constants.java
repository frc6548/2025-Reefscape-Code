// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {
    public static final class AprilTagConstants {
        public static int REEF_AB_TAGID;
        public static int REEF_CD_TAGID;
        public static int REEF_EF_TAGID;
        public static int REEF_GH_TAGID;
        public static int REEF_IJ_TAGID;
        public static int REEF_KL_TAGID;

        public static int CORAL_STATION_LEFT_TAGID;
        public static int CORAL_STATION_RIGHT_TAGID;

        // below values are in meters
        public static final double INSIDE_REEF_ZONE_THRESHOLD = 1.6;
        public static final double AUTO_ADJUST_THRESHOLD = 1.8;

        private static final double CORAL_STATION_OFFSET_HORIZONTAL = 0.3;
        private static final double CORAL_STATION_OFFSET_VERTICAL = 0.3;
        public static Translation2d CORAL_STATION_LEFT_OFFSET;
        public static Translation2d CORAL_STATION_RIGHT_OFFSET;

        public static void update(Alliance alliance) {
            REEF_AB_TAGID = alliance == Alliance.Blue ? 18 : 7;
            REEF_CD_TAGID = alliance == Alliance.Blue ? 17 : 8;
            REEF_EF_TAGID = alliance == Alliance.Blue ? 22 : 9;
            REEF_GH_TAGID = alliance == Alliance.Blue ? 21 : 10;
            REEF_IJ_TAGID = alliance == Alliance.Blue ? 20 : 11;
            REEF_KL_TAGID = alliance == Alliance.Blue ? 19 : 6;

            CORAL_STATION_LEFT_TAGID = alliance == Alliance.Blue ? 13 : 1;
            CORAL_STATION_RIGHT_TAGID = alliance == Alliance.Blue ? 12 : 2;

            CORAL_STATION_LEFT_OFFSET = alliance == Alliance.Blue ?
                new Translation2d(CORAL_STATION_OFFSET_HORIZONTAL, -CORAL_STATION_OFFSET_VERTICAL) :
                new Translation2d(CORAL_STATION_OFFSET_HORIZONTAL, CORAL_STATION_OFFSET_VERTICAL);
            CORAL_STATION_RIGHT_OFFSET = alliance == Alliance.Blue ?
                new Translation2d(-CORAL_STATION_OFFSET_HORIZONTAL, -CORAL_STATION_OFFSET_VERTICAL) :
                new Translation2d(CORAL_STATION_OFFSET_HORIZONTAL, CORAL_STATION_OFFSET_VERTICAL);
        }
    }

    // AUTOMATIC_GO_TO_POSITION_TIMEOUT_SECONDS; should be slightly above actual time needed
    public static final double AUTOMATIC_GO_TO_POSITION_TIMEOUT_SECONDS = 2;
    public static final double TIME_UNTIL_CORAL_IS_SCORED_SECONDS = 0.25;
}
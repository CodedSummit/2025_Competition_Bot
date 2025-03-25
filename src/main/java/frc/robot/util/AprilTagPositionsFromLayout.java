package frc.robot.util;
import static edu.wpi.first.math.util.Units.degreesToRadians;

// Courtesy of team 7414.   See https://www.chiefdelphi.com/t/alignment-pathfinder/492932/15
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class AprilTagPositionsFromLayout {
        private static final HashMap<Integer, Pose2d> WELDED_APRIL_TAG_POSITIONS = new HashMap<>();
        private static final HashMap<Integer, Pose2d> WELDED_RED_CORAL_APRIL_TAG_POSITIONS = new HashMap<>();
        private static final HashMap<Integer, Pose2d> WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS = new HashMap<>();
        static AprilTagFieldLayout m_CompetitionAprilTagFieldLayout = AprilTagFields.k2025ReefscapeWelded
                        .loadAprilTagLayoutField();
        
        private static  AprilTagPositionsFromLayout theLayout = null;

        private static  AprilTagPositionsFromLayout init() {
                if (theLayout == null){
                        theLayout = new AprilTagPositionsFromLayout();
                }
                return theLayout;
        }

        public static HashMap<Integer, Pose2d> weldedAprilTagPositions() {
                init();
                return WELDED_APRIL_TAG_POSITIONS;
        }

        public static HashMap<Integer, Pose2d> weldedRedAprilTagPositions() {
                init();
                return WELDED_RED_CORAL_APRIL_TAG_POSITIONS;
        }

        public static HashMap<Integer, Pose2d> weldedBlueAprilTagPositions() {
                init();
                return WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS;
        }




        private AprilTagPositionsFromLayout() {

                List<AprilTag> allTags = m_CompetitionAprilTagFieldLayout.getTags();
                Rotation2d deg180 = new Rotation2d(degreesToRadians(180.0));
                allTags.forEach(
                                (tag) -> {
                                        Pose2d aPose = tag.pose.toPose2d();
                                        aPose.rotateBy(deg180);
                                        WELDED_APRIL_TAG_POSITIONS.put(tag.ID,
                                                        new Pose2d(aPose.getX(), aPose.getY(), aPose.getRotation()));
                                });

                WELDED_RED_CORAL_APRIL_TAG_POSITIONS.put(6, WELDED_APRIL_TAG_POSITIONS.get(6));
                WELDED_RED_CORAL_APRIL_TAG_POSITIONS.put(7, WELDED_APRIL_TAG_POSITIONS.get(7));
                WELDED_RED_CORAL_APRIL_TAG_POSITIONS.put(8, WELDED_APRIL_TAG_POSITIONS.get(8));
                WELDED_RED_CORAL_APRIL_TAG_POSITIONS.put(9, WELDED_APRIL_TAG_POSITIONS.get(9));
                WELDED_RED_CORAL_APRIL_TAG_POSITIONS.put(10, WELDED_APRIL_TAG_POSITIONS.get(10));
                WELDED_RED_CORAL_APRIL_TAG_POSITIONS.put(11, WELDED_APRIL_TAG_POSITIONS.get(11));

                WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS.put(17, WELDED_APRIL_TAG_POSITIONS.get(17));
                WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS.put(18, WELDED_APRIL_TAG_POSITIONS.get(18));
                WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS.put(19, WELDED_APRIL_TAG_POSITIONS.get(19));
                WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS.put(20, WELDED_APRIL_TAG_POSITIONS.get(20));
                WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS.put(21, WELDED_APRIL_TAG_POSITIONS.get(21));
                WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS.put(22, WELDED_APRIL_TAG_POSITIONS.get(22));
        }

}
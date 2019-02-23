package frc.robot.driveutil;

import java.util.List;

/**
 * Collection of all the profiles we use for autonomous
 */
public class DriveProfiles {

    public static List<TJDriveMotionPoint> straightTest;

    public static void init() {
        straightTest = TJDriveMotion.loadStaticProfile("straightTest", false);
    }

}
package frc.robot.driveutil;

import java.util.List;

/**
 * Collection of all the profiles we use for autonomous
 */
public class DriveProfiles {

    public static List<TJDriveMotionPoint> straightTest;
    public static List<TJDriveMotionPoint> leftTurnTest;
    public static List<TJDriveMotionPoint> rightTurnTest;

    public static void init() {
        straightTest = TJDriveMotion.loadStaticProfile("StraightTest", false);
        leftTurnTest = TJDriveMotion.loadStaticProfile("LeftTurnTest", false);
        rightTurnTest = TJDriveMotion.loadStaticProfile("RightTurnTest", false);
    }

}
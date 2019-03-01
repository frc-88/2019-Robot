package frc.robot.driveutil;

import java.util.List;

/**
 * Collection of all the profiles we use for autonomous
 */
public class DriveProfiles {

    public static List<TJDriveMotionPoint> straight8;
    public static List<TJDriveMotionPoint> leftTurnTest;
    public static List<TJDriveMotionPoint> rightTurnTest;
    public static List<TJDriveMotionPoint> leftPark;
    public static List<TJDriveMotionPoint> rightPark;

    public static void init() {
        straight8 = TJDriveMotion.loadStaticProfile("StraightTest", false);
        leftTurnTest = TJDriveMotion.loadStaticProfile("LeftTurnTest", false);
        rightTurnTest = TJDriveMotion.loadStaticProfile("RightTurnTest", false);
        leftPark = TJDriveMotion.loadStaticProfile("LeftPark", false);
        rightPark = TJDriveMotion.loadStaticProfile("RightPark", false);
    }

}
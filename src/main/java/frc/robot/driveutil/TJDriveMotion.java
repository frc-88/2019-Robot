/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.driveutil;

import java.io.File;
import java.util.ArrayList;
import java.util.Scanner;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Robot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * Add your docs here.
 */
public class TJDriveMotion {
    private static final File home = new File(Filesystem.getDeployDirectory(), "output");
    private static final int NUM_OF_READS = 8;

    public TJDriveMotion() {
    }

    public static ArrayList<TJDriveMotionPoint> loadStaticProfile(String profile, Boolean reverse) {
        ArrayList<TJDriveMotionPoint> points = new ArrayList<TJDriveMotionPoint>();

        try {
            Scanner main = new Scanner(new File(home, profile + ".pf1.csv"));
            Scanner left = new Scanner(new File(home, profile + ".left.pf1.csv"));
            Scanner right = new Scanner(new File(home, profile + ".right.pf1.csv"));

            main.useDelimiter("[,\n]");
            left.useDelimiter("[,\n]");
            right.useDelimiter("[,\n]");

            main.nextLine();
            left.nextLine();
            right.nextLine();

            // puts file values in ArrayList
            while (main.hasNextLine() && left.hasNextLine() && right.hasNextLine()) {
                ArrayList<Double> mainList = new ArrayList<Double>();
                ArrayList<Double> leftList = new ArrayList<Double>();
                ArrayList<Double> rightList = new ArrayList<Double>();
                for(int i = 0; i < NUM_OF_READS; i++){
                    mainList.add(Double.parseDouble(main.next()));
                }

                for(int i = 0; i < NUM_OF_READS; i++){
                    leftList.add(Double.parseDouble(left.next()));
                }

                for(int i = 0; i < NUM_OF_READS; i++){
                    rightList.add(Double.parseDouble(right.next()));
                }

                TJDriveMotionPoint point = new TJDriveMotionPoint();
                point.dt = mainList.get(0);
                point.x = mainList.get(1);
                point.y = mainList.get(2);
                point.heading = mainList.get(7);
                point.leftPosition = leftList.get(3);
                point.leftVelocity = leftList.get(4);
                point.leftAcceleration = leftList.get(5);
                point.leftJerk = leftList.get(6);
                point.rightPosition = rightList.get(3);
                point.rightVelocity = rightList.get(4);
                point.rightAcceleration = rightList.get(5);
                point.rightJerk = rightList.get(6);

                points.add(point);


            }
            main.close();
            left.close();
            right.close();
        } catch (Exception e) {
            // TODO: handle file not found
            e.printStackTrace();
        }

        return points;
    }
}

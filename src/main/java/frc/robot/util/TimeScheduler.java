package frc.robot.util;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.RobotController;

/**
 * Class which runs a through a list of functions, stopping once a certain 
 * ammount of time has passed. Useful for things like reading talon values 
 * for the dashboard which take about 100ns to 5ms but don't need to run on
 * every loop.
 */
public class TimeScheduler {

    private ArrayList<Runnable> functions;
    private int curIndex = 0;

    public TimeScheduler() {
        functions = new ArrayList<>();
    }

    /**
     * Add the given function to the list of functions that are scheduled.
     */
    public void addFunction(Runnable func) {
        functions.add(func);
    }

    /**
     * Runs through the functions for the given ammount of time (in us).
     */
    public void run(long time) {
        long startTime = RobotController.getFPGATime();
        long startIndex = curIndex;

        do {
            
            functions.get(curIndex).run();

            curIndex++;
        } while (RobotController.getFPGATime() - startTime < time
                && curIndex != startIndex);
    }
}
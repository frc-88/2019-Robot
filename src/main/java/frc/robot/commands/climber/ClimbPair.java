package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;

public class ClimbPair {
    public Command prep;
    public Command climb;

    public ClimbPair(Command prep, Command climb) {
        this.prep = prep;
        this.climb = climb;
    }
}
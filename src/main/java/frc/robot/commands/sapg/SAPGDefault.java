/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.sapg;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class SAPGDefault extends Command {
  private static final int COUNTS_TO_CLOSE = 10;

  private int noPanelCounts = 0;

  public SAPGDefault() {
    requires(Robot.m_sapg);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // This command will be interrupted often by other SAPG commands (most InstantCommands)
    // remember that initialize will be called each time this command starts again after
    // being interrupted. So...let's not re-initialize our counts and just keep them over time.
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (Robot.m_sapg.hasPanel()) {
      noPanelCounts = 0;
    } else {
      noPanelCounts++;
    }

    if (noPanelCounts >= COUNTS_TO_CLOSE) {
      //      Robot.m_sapg.close();
    }

    //    Robot.m_sapg.retract();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

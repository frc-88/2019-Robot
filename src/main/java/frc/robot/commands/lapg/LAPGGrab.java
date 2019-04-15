/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.lapg;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class LAPGGrab extends Command {
  private static final long DEFAULT_DELAY = 100000; // microseconds
  private static final long ACTIVE_TIME = 600000; // microseconds
  private static final long OPEN_TIME = 600000; // microseconds

  private int state;
  private long startTime;

  public LAPGGrab() {
    requires(Robot.m_lapg);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    state = 0;
    startTime = RobotController.getFPGATime();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    switch (state) {
    case 0:
      //close
      Robot.m_lapg.close();
      gotoNextStateAfterDelay();
      break;
    case 1:
      // neutral, wait for switch
      Robot.m_lapg.neutral();
      if (Robot.m_lapg.getSwitch()) {
        state++;
      }
      break;
    case 2:
      // active
      Robot.m_lapg.active();
      gotoNextStateAfterDelay(ACTIVE_TIME);
      break;
    case 3:
      // open
      Robot.m_lapg.open();
      gotoNextStateAfterDelay(OPEN_TIME);
      break;
    case 4:
      // retract
      Robot.m_lapg.retract();
      state = 99;
      break;
    default:
      state = 99;
      break;
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return state == 99;
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

  private void gotoNextStateAfterDelay() {
    gotoNextStateAfterDelay(DEFAULT_DELAY);
  }

  private void gotoNextStateAfterDelay(long delay) {
    if (RobotController.getFPGATime() - startTime > delay) {
      state++;
      startTime = RobotController.getFPGATime();
    }

  }
}

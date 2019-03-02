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
  double kP = -1.0;
  double error;

  private int noPanelCounts = 0;
  private final int COUNTS_TO_CLOSE = 10;

  public SAPGDefault() {
    requires(Robot.m_sapg);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    noPanelCounts = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (!Robot.m_sapg.getPIDController().isEnabled()) {
      error = Robot.m_sapg.getNormalizedPosition();
      Robot.m_sapg.set(kP * error);
    } else {
      Robot.m_sapg.set(0.0);
    }

    if (!Robot.m_sapg.hasPanel()) {
      noPanelCounts++;
    } else {
      noPanelCounts = 0;
    }

    if (noPanelCounts >= COUNTS_TO_CLOSE) {
      Robot.m_sapg.closeTheJaws();
    }

    Robot.m_sapg.reversePush();
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

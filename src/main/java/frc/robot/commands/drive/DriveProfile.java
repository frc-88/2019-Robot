/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.util.List;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.driveutil.TJDriveMotionPoint;

public class DriveProfile extends Command {

  private List<TJDriveMotionPoint> profile;

  public DriveProfile(List<TJDriveMotionPoint> profile) {
    requires(Robot.m_drive);
    this.profile = profile;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_drive.loadMotionProfile(profile);
    Robot.m_drive.runMotionProfile();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.m_drive.isProfileFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_drive.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

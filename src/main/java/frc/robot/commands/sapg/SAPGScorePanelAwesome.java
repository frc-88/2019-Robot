/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.sapg;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class SAPGScorePanelAwesome extends Command {

  private int state;
  private long startTime;
  private final long PUSH_TIME = 500000; // microseconds
  private final long CLOSE_TIME = 500000; // microseconds

  public SAPGScorePanelAwesome() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_sapg);
    requires(Robot.m_drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
      state=0;
      startTime=RobotController.getFPGATime();
      Robot.m_drive.arcadeDrive(0, 0);
      Robot.m_limelight_sapg.ledOff();
      Robot.m_limelight_sapg.camDriver();

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    switch (state) {
    case 0:
      // open, push out
      Robot.m_drive.arcadeDrive(0, 0);
      Robot.m_sapg.deploy();
      Robot.m_sapg.open();
      if (RobotController.getFPGATime() - startTime > PUSH_TIME) {
        state++;
        startTime = RobotController.getFPGATime();

      }
      break;

    case 1:
      // close
      Robot.m_drive.arcadeDrive(0, 0);
      Robot.m_sapg.close();
      if (RobotController.getFPGATime() - startTime > CLOSE_TIME) {
        state++;
        startTime = RobotController.getFPGATime();
      }
      break;

    case 2:
      // pull in
      Robot.m_drive.arcadeDrive(0, 0);
      Robot.m_sapg.retract();
      if (RobotController.getFPGATime() - startTime > PUSH_TIME) {
        state++;
        startTime = RobotController.getFPGATime();
      }
      break;
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return state == 3;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_drive.arcadeDrive(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

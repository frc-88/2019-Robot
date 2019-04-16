/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * An example command.  You can replace me with your own command.
 */
public class ArcadeDrive extends Command {
  public ArcadeDrive() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_drive.resetVelocityPID();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_drive.autoshift();
    
    double speed=Robot.m_oi.getDriverLeftYAxis();
    double turn=Robot.m_oi.getDriverRightXAxis();

    if (Robot.m_oi.isDriverButtonAPressed()) {
      // titan:
      // turn = 0.8 * Robot.m_limelight.turnToTarget();

      // jupiter:
      turn = 0.6 * Robot.m_limelight.turnToTarget();
    }

    if (Robot.m_oi.isDriverButtonBPressed()) {
      // titan:
      turn = 0.6 * Robot.m_limelight.turnToTarget();

      // jupiter:
      turn = 0.4 * Robot.m_limelight.turnToTarget();

      if (Robot.m_limelight.hasTarget()) {
        double adjustment = Math.min(Robot.m_limelight.getTargetArea() / RobotMap.LIMELIGHT_MAX_AREA, 1);
        speed = RobotMap.TARGETING_MAX_SPEED - (adjustment * (RobotMap.TARGETING_MAX_SPEED - RobotMap.TARGETING_MIN_SPEED));
        speed *= -1;
      }
    }


    Robot.m_drive.arcadeDrive(speed, turn);
  }
  

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_drive.basicDrive(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class ClimberMoveShoulder extends Command {

  double shoulderTarget;

  public ClimberMoveShoulder() {
    requires(Robot.m_climber);
    requires(Robot.m_arm);
    SmartDashboard.putNumber("SetClimberPosition", 0);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_arm.configureCoastMode();

    shoulderTarget = SmartDashboard.getNumber("SetClimberPosition", 0);
    Robot.m_climber.moveShoulder(shoulderTarget);
    if (shoulderTarget < Robot.m_arm.getShoulderAbsDegrees()) {
      Robot.m_arm.setShoulderVoltage(-0.05);
    } else {
      Robot.m_arm.setShoulderVoltage(0);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (shoulderTarget < Robot.m_arm.getShoulderAbsDegrees()) {
      Robot.m_arm.setShoulderVoltage(-0.05);
    } else {
      Robot.m_arm.setShoulderVoltage(0);
    }
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

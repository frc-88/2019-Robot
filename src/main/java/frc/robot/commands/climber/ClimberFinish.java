/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;

public class ClimberFinish extends Command {

  private Climber climber = Robot.m_climber;
  private Arm arm = Robot.m_arm;

  private final int CLIMBER_TARGET = 21000;
  private final double SHOULDER_TARGET = 121;
  private final double ELBOW_TARGET = 180;

  public ClimberFinish() {
    requires(climber);
    requires(arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    climber.configForEncoderPID();
    climber.moveEncoder(CLIMBER_TARGET);
    arm.moveShoulder(SHOULDER_TARGET);
    arm.moveElbow(ELBOW_TARGET);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
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

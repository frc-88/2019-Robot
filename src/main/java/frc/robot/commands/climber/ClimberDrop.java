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

public class ClimberDrop extends Command {

  private Climber climber = Robot.m_climber;
  private Arm arm = Robot.m_arm;

  private final double SHOULDER_TARGET = 121;
  private final int CLIMBER_LIFT_AMOUNT = 500;

  public ClimberDrop() {
    requires(climber);
    requires(arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    arm.setShoulderSpeed(50);
    arm.moveShoulder(SHOULDER_TARGET);
    climber.configForEncoderPID();
    climber.moveEncoder(climber.getPosition() - CLIMBER_LIFT_AMOUNT);
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

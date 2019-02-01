/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ArmGoToPosition extends Command {
  private double shoulder_target;
  private double elbow_target;

  public ArmGoToPosition(double shoulder, double elbow) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_arm);
    shoulder_target = shoulder;
    elbow_target = elbow;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_arm.moveShoulder(shoulder_target);
    Robot.m_arm.moveElbow(elbow_target);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(Robot.m_arm.getShoulderPosition() - shoulder_target) < RobotMap.ARM_TOLERANCE
        && Math.abs(Robot.m_arm.getElbowPosition() - elbow_target) < RobotMap.ARM_TOLERANCE;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_arm.stopArm();
  }
}

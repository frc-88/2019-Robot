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
import frc.robot.subsystems.Arm;
import frc.robot.util.ArmPosition;
import frc.robot.util.ArmSetpoint;

public class ArmGoToSetpoint extends Command {

  private Arm arm = Robot.m_arm;

  private ArmSetpoint start;
  private ArmSetpoint target;
  private ArmSetpoint[] path;
  private int currentPathSetpoint = 0;

  public ArmGoToSetpoint(ArmSetpoint setpoint) {
    requires(Robot.m_arm);
    target = setpoint;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    start = arm.getCurrentSetpoint();
    path = ArmPosition.getPath(start, target);
    currentPathSetpoint = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (currentPathSetpoint < path.length) {
      if (currentPathSetpoint == 0) {
        arm.setSetpoint(start, path[currentPathSetpoint]);
      } else {
        arm.setSetpoint(path[currentPathSetpoint-1], path[currentPathSetpoint]);
      }

      if (Math.abs(arm.getShoulderMotorDegrees() - path[currentPathSetpoint].shoulder) < RobotMap.ARM_TOLERANCE
          && Math.abs(arm.getElbowMotorDegrees() - path[currentPathSetpoint].elbow) < RobotMap.ARM_TOLERANCE) {
        currentPathSetpoint++;
      }

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

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
  private boolean elbowLock;
  private double elbowOffset;

  public ArmGoToSetpoint(ArmSetpoint setpoint) {
    requires(Robot.m_arm);
    target = setpoint;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    start = arm.getCurrentSetpoint();
    path = ArmPosition.getPath(start, target);
    currentPathSetpoint = 0;
    elbowLock = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (currentPathSetpoint < path.length) {

      ArmSetpoint to = path[currentPathSetpoint];
      ArmSetpoint from;
      if (currentPathSetpoint == 0) {
        from = start;
      } else {
        from = path[currentPathSetpoint-1];
      }

      if (to.lockElbow && !elbowLock) {
        elbowLock = true;
        elbowOffset = arm.getShoulderMotorDegrees() - arm.getElbowMotorDegrees();
      }

      double shoulderSpeed;
      double elbowSpeed;

      double shoulderDist = Math.abs(from.shoulder - to.shoulder);
      double elbowDist = Math.abs(from.elbow - to.elbow);

      if (elbowLock) {
        shoulderSpeed = Math.min(RobotMap.SHOULDER_MAX_SPEED, RobotMap.ELBOW_MAX_SPEED);
        elbowSpeed = shoulderSpeed;
      } else if (shoulderDist / RobotMap.SHOULDER_MAX_SPEED > elbowDist / RobotMap.ELBOW_MAX_SPEED) {
        shoulderSpeed = RobotMap.SHOULDER_MAX_SPEED;
        elbowSpeed = RobotMap.SHOULDER_MAX_SPEED * elbowDist / shoulderDist;
      } else {
        elbowSpeed = RobotMap.ELBOW_MAX_SPEED;
        shoulderSpeed = RobotMap.ELBOW_MAX_SPEED * shoulderDist / elbowDist;
      }


      int passIdx = currentPathSetpoint;
      while (path[passIdx].passShoulder) {
        //System.out.println("Shoulder pass");
        passIdx++;
      }
      double shoulderTarget = path[passIdx].shoulder;

      double elbowTarget;
      if (elbowLock) {
        elbowTarget = arm.getShoulderMotorDegrees() - elbowOffset;
      } else {
        passIdx = currentPathSetpoint;
        while (path[passIdx].passElbow) {
          passIdx++;
        }
        elbowTarget = path[passIdx].elbow;
      }

      arm.setSetpoint(to, shoulderTarget, elbowTarget, shoulderSpeed, elbowSpeed);


      boolean shoulderOnTarget = true;
      if (from.shoulder < to.shoulder) {
        shoulderOnTarget = arm.getShoulderMotorDegrees() > to.shoulder - RobotMap.ARM_TOLERANCE;
      } else if (from.shoulder > to.shoulder) {
        shoulderOnTarget = arm.getShoulderMotorDegrees() < to.shoulder + RobotMap.ARM_TOLERANCE;
      }

      boolean elbowOnTarget = true;
      if (from.elbow < to.elbow) {
        elbowOnTarget = arm.getElbowMotorDegrees() > to.elbow - RobotMap.ARM_TOLERANCE;
      } else if (from.elbow > to.elbow) {
        elbowOnTarget = arm.getElbowMotorDegrees() < to.elbow + RobotMap.ARM_TOLERANCE;
      }

      if (shoulderOnTarget && elbowOnTarget) {
        currentPathSetpoint++;
        elbowLock = false;
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

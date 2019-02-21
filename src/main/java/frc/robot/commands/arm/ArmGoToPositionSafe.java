/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ArmGoToPositionSafe extends Command {
  private double shoulderTarget;
  private double elbowTarget;
  private double shoulderTargetDegrees;
  private double elbowTargetDegrees;
  private boolean usePreferences;
  private boolean targetSafe;

  public ArmGoToPositionSafe() {
    requires(Robot.m_arm);
    usePreferences = true;
  }

  public ArmGoToPositionSafe(double [] target) {
    requires(Robot.m_arm);
    saveArguments(target[0], target[1]);
  }

  public ArmGoToPositionSafe(double shoulder, double elbow) {
    requires(Robot.m_arm);
    saveArguments(shoulder, elbow);
  }

  private void saveArguments(double shoulder, double elbow) {
    usePreferences = false;
    shoulderTargetDegrees = shoulder;
    shoulderTarget = Robot.m_arm.convertShoulderDegreesToMotor(shoulderTargetDegrees);
    elbowTargetDegrees = elbow;
    elbowTarget = Robot.m_arm.convertElbowDegreesToMotor(elbowTargetDegrees);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (usePreferences) {
      Preferences prefs = Preferences.getInstance();

      shoulderTargetDegrees = prefs.getDouble("ArmShoulderTarget", 0.0);
      elbowTargetDegrees = prefs.getDouble("ArmElbowTarget",0.0);
      shoulderTarget = Robot.m_arm.convertShoulderDegreesToMotor(shoulderTargetDegrees);
      elbowTarget = Robot.m_arm.convertElbowDegreesToMotor(elbowTargetDegrees);
    }

    targetSafe = Robot.m_arm.isSafePosition(shoulderTargetDegrees, elbowTargetDegrees);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (targetSafe) {
      // what if we only did this every 10 cycles?
      Robot.m_arm.moveShoulder(shoulderTarget);
      Robot.m_arm.moveElbow(elbowTarget);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !Robot.m_arm.isSafePosition() || !targetSafe;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_arm.stopArm();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_arm.stopArm();
  }
}

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
  private double shoulder_target;
  private double elbow_target;
  private double shoulder_degrees;
  private double elbow_degrees;
  private boolean usePreferences;
  private String targetPosition;
  private boolean targetSafe;

  public ArmGoToPositionSafe() {
    requires(Robot.m_arm);
    usePreferences = true;
  }

  public ArmGoToPositionSafe(String position) {
    requires(Robot.m_arm);

    targetPosition = position;
    usePreferences = true;
  }

  public ArmGoToPositionSafe(double [] position) {
    requires(Robot.m_arm);

    shoulder_degrees = position[0];
    elbow_degrees = position[1];

    usePreferences = false;
  }

  public ArmGoToPositionSafe(double shoulder, double elbow) {
    requires(Robot.m_arm);


    shoulder_degrees = shoulder;
    elbow_degrees = elbow;

    usePreferences = false;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (usePreferences) {
      Preferences prefs = Preferences.getInstance();
      String elbowTarget, shoulderTarget;

      if (targetPosition != null) {
        shoulderTarget = "Arm:" + targetPosition + "_shoulder";
        elbowTarget = "Arm:" + targetPosition + "_elbow";
      } else {
        shoulderTarget = "Arm:ShoulderTarget";
        elbowTarget = "Arm:ElbowTarget";
      }

      shoulder_target = Robot.m_arm.convertShoulderDegreesToMotor(prefs.getDouble(shoulderTarget, 0.0));
      elbow_target = Robot.m_arm.convertElbowDegreesToMotor(prefs.getDouble(elbowTarget,0.0));
    } else {
      shoulder_target = Robot.m_arm.convertShoulderDegreesToMotor(shoulder_degrees);
      elbow_target = Robot.m_arm.convertElbowDegreesToMotor(elbow_degrees);
    }

    targetSafe = Robot.m_arm.isSafePosition(shoulder_degrees, elbow_degrees);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (targetSafe) {
      // what if we only did this every 10 cycles?
      Robot.m_arm.moveShoulder(shoulder_target);
      Robot.m_arm.moveElbow(elbow_target);
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

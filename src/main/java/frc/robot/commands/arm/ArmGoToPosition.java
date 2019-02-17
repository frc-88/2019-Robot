/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ArmGoToPosition extends Command {
  private double shoulder_target;
  private double elbow_target;
  private boolean usePrefences;

  public ArmGoToPosition() {
    requires(Robot.m_arm);

    usePrefences = true;
  }

  public ArmGoToPosition(double shoulder, double elbow) {
    requires(Robot.m_arm);

    usePrefences = false;
    shoulder_target = Robot.m_arm.convertShoulderDegreesToMotor(shoulder);
    elbow_target = Robot.m_arm.convertElbowDegreesToMotor(elbow);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (usePrefences) {
      Preferences prefs = Preferences.getInstance();

      shoulder_target = Robot.m_arm.convertShoulderDegreesToMotor(prefs.getDouble("ArmShoulderTarget", 0.0));
      elbow_target = Robot.m_arm.convertElbowDegreesToMotor(prefs.getDouble("ArmElbowTarget",0.0));
    }

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_arm.moveShoulder(shoulder_target);
    Robot.m_arm.moveElbow(elbow_target);

    SmartDashboard.putNumber("Arm:commandShoulder", shoulder_target);
    SmartDashboard.putNumber("Arm:commandElbow", elbow_target);

    

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

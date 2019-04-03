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
import frc.robot.util.ArmSetpoint;

public class ArmGoToPosition extends Command {
  private double shoulder_target;
  private double elbow_target;
  private double shoulder_degrees;
  private double elbow_degrees;
  private boolean usePrefences;
  private String targetPosition;
  private boolean goingToIntake=false;
  public ArmGoToPosition() {
    requires(Robot.m_arm);

    usePrefences = true;
  }

  public ArmGoToPosition(String position) {
    requires(Robot.m_arm);
  if(position.equals("intake")){
  goingToIntake=true;
}
    targetPosition = position;
    usePrefences = true;
  }

  public ArmGoToPosition(ArmSetpoint position) {
    requires(Robot.m_arm);

    shoulder_degrees = position.shoulder;
    elbow_degrees = position.elbow;

    usePrefences = false;
  }

  public ArmGoToPosition(double shoulder, double elbow) {
    requires(Robot.m_arm);

    shoulder_degrees = shoulder;
    elbow_degrees = elbow;

    usePrefences = false;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (usePrefences) {
      Preferences prefs = Preferences.getInstance();
      String elbowTarget, shoulderTarget;

      if (targetPosition != null) {
        shoulderTarget = "Arm:" + targetPosition + "_shoulder";
        elbowTarget = "Arm:" + targetPosition + "_elbow";
      } else {
        shoulderTarget = "Arm:ShoulderTarget";
        elbowTarget = "Arm:ElbowTarget";
      }

      shoulder_target = prefs.getDouble(shoulderTarget, 0.0);
      elbow_target = prefs.getDouble(elbowTarget,0.0);
    } else {
      shoulder_target = shoulder_degrees;
      elbow_target = elbow_degrees;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_arm.moveShoulder(shoulder_target);
    if(goingToIntake && Robot.m_arm.getShoulderAbsDegrees()>135){
    Robot.m_arm.moveElbow(elbow_target);

    }
    else if(!goingToIntake){
          Robot.m_arm.moveElbow(elbow_target);

    }
    else if(goingToIntake && Robot.m_arm.getShoulderAbsDegrees()<135){
      Robot.m_arm.moveElbow(0);
    }

    SmartDashboard.putNumber("Arm:commandShoulder", shoulder_target);
    SmartDashboard.putNumber("Arm:commandElbow", elbow_target);

    

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // return Math.abs(Robot.m_arm.getShoulderPosition() - shoulder_target) < RobotMap.ARM_TOLERANCE
    //     && Math.abs(Robot.m_arm.getElbowPosition() - elbow_target) < RobotMap.ARM_TOLERANCE;
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
    Robot.m_arm.stopArm();
  }
}

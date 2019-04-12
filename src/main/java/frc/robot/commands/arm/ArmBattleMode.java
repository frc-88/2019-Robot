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

public class ArmBattleMode extends Command {

  private double shoulderSpeed = 0.2;
  private double elbowSpeed = 0.2;

  public ArmBattleMode() {
    requires(Robot.m_arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Preferences prefs = Preferences.getInstance();
    if (prefs.containsKey("Arm:BattleModeShoulder")) {
      shoulderSpeed = prefs.getDouble("Arm:BattleModeShoulder", shoulderSpeed);
    } else {
      prefs.putDouble("Arm:BattleModeShoulder", shoulderSpeed);
    }
    if (prefs.containsKey("Arm:BattleModeElbow")) {
      elbowSpeed = prefs.getDouble("Arm:BattleModeElbow", elbowSpeed);
    } else {
      prefs.putDouble("Arm:BattleModeElbow", elbowSpeed);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_arm.setShoulderVoltage(shoulderSpeed);
    Robot.m_arm.setElbowVoltage(elbowSpeed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
    end();
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;

public class ClimberPrep extends Command {

  private Climber climber = Robot.m_climber;

  private int WINCH_DISTANCE = 35000; //2100 on jupiter

  public ClimberPrep() {
    requires(climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Preferences prefs = Preferences.getInstance();
    if (prefs.containsKey("Climber:PrepTarget")) {
      WINCH_DISTANCE = prefs.getInt("Climber:PrepTarget", WINCH_DISTANCE);
    } else {
      prefs.putInt("Climber:PrepTarget", WINCH_DISTANCE);
    }

    climber.configForEncoderPID();
    climber.zeroEncoder();
    climber.moveEncoder(WINCH_DISTANCE);
    climber.prep();
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

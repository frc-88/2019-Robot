/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.Robot;
import frc.robot.util.ArmPosition;

public class InPreClimbCommand extends ConditionalCommand {

	public InPreClimbCommand(Command inPreClimb) {
		super(inPreClimb);
	}

	public InPreClimbCommand(Command inPreClimb, Command notPreClimb) {
		super(inPreClimb, notPreClimb);
	}

	public InPreClimbCommand(String name, Command inPreClimb) {
		super(name, inPreClimb);
	}

	public InPreClimbCommand(String name, Command inPreClimb, Command notPreClimb) {
		super(name, inPreClimb, notPreClimb);
	}

  @Override
  protected boolean condition() {
    return Robot.m_climber.isPrepped();
  }
}

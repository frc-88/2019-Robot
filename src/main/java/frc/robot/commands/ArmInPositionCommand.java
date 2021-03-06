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
import frc.robot.util.ArmSetpoint;

public class ArmInPositionCommand extends ConditionalCommand {

	private ArmSetpoint m_position;

	public ArmInPositionCommand(ArmSetpoint position, Command inPosition) {
		super(inPosition);
		m_position = position;
	}

	public ArmInPositionCommand(ArmSetpoint position, Command inPosition, Command notInPosition) {
		super(inPosition, notInPosition);
		m_position = position;
	}

	public ArmInPositionCommand(String name, ArmSetpoint position, Command inPosition) {
		super(name, inPosition);
		m_position = position;
	}

	public ArmInPositionCommand(String name, ArmSetpoint position, Command inPosition, Command notInPosition) {
		super(name, inPosition, notInPosition);
		m_position = position;
	}

  @Override
  protected boolean condition() {
    return Robot.m_arm.getCurrentSetpoint().equals(m_position);
  }
}

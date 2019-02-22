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
import frc.robot.commands.arm.ArmGoToPositionSafe;

public class ArmGTP extends ConditionalCommand {
	private double[] m_target;

	public ArmGTP(double[] target) {
		super(new ArmGoToPositionSafe(target));
		m_target = target;
	}

	public ArmGTP(double[] target, Command safe) {
		super(safe);
		m_target = target;
	}

	public ArmGTP(double[] target, Command safe, Command dangerous) {
		super(safe, dangerous);
		m_target = target;
	}

	public ArmGTP(double[] target, String name, Command safe) {
		super(name, safe);
		m_target = target;
	}

	public ArmGTP(double[] target, String name, Command safe, Command dangerous) {
		super(name, safe, dangerous);
		m_target = target;
	}

	@Override
	protected boolean condition() {
		return Robot.m_arm.isSafePosition(m_target);
	}
}

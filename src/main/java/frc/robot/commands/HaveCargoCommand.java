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

public class HaveCargoCommand extends ConditionalCommand {

	public HaveCargoCommand(Command haveCargo) {
		super(haveCargo);
	}

	public HaveCargoCommand(Command haveCargo, Command noCargo) {
		super(haveCargo, noCargo);
	}

	public HaveCargoCommand(String name, Command haveCargo) {
		super(name, haveCargo);
	}

	public HaveCargoCommand(String name, Command haveCargo, Command noCargo) {
		super(name, haveCargo, noCargo);
	}

  @Override
  protected boolean condition() {
    return Robot.m_intake.hasCargo();
  }
}

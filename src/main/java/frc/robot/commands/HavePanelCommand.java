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

public class HavePanelCommand extends ConditionalCommand {

	public HavePanelCommand(Command havePanel) {
		super(havePanel);
	}

	public HavePanelCommand(Command havePanel, Command noPanel) {
		super(havePanel, noPanel);
	}

	public HavePanelCommand(String name, Command havePanel) {
		super(name, havePanel);
	}

	public HavePanelCommand(String name, Command havePanel, Command noPanel) {
		super(name, havePanel, noPanel);
	}

  @Override
  protected boolean condition() {
    return Robot.m_lapg.hasPanel();
  }
}

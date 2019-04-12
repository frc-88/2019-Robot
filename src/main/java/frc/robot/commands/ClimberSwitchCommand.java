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
import frc.robot.commands.ClimberSelectedCommand.SelectState;

public class ClimberSwitchCommand extends ConditionalCommand {

    public ClimberSwitchCommand(Command switchOff) {
		super(switchOff);
	}

	public ClimberSwitchCommand(Command switchOff, Command switchOn) {
		super(switchOff, switchOn);
	}

	public ClimberSwitchCommand(String name, Command switchOff) {
		super(name, switchOff);
	}

	public ClimberSwitchCommand(String name, Command switchOff, Command switchOn) {
		super(name, switchOff, switchOn);
	}

    @Override
    protected boolean condition() {
		if (Robot.m_oi.inLevel3Mode()) {
			ClimberSelectedCommand.selectState = SelectState.OFF;
			return true;
		} else {
			ClimberSelectedCommand.selectState = SelectState.ON;
			return false;
		}
        
    }


}

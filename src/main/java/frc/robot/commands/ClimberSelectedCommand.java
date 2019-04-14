/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class ClimberSelectedCommand extends InstantCommand {

	public enum SelectState {
		UNPREPPED,
		OFF,
		ON
	}

	public static SelectState selectState = SelectState.UNPREPPED;

	private Command switchOffCommand;
	private Command switchOnCommand;

    public ClimberSelectedCommand(Command switchOff, Command switchOn) {
		this.switchOffCommand = switchOff;
		this.switchOnCommand = switchOn;
	}

	@Override
	public void initialize() {
		switch (selectState) {
			case UNPREPPED:
				break;
			
			case OFF:
				switchOffCommand.start();
				break;

			case ON:
				switchOnCommand.start();
				break;
		}
	}

}

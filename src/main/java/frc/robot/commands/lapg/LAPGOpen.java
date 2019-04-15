/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.lapg;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class LAPGOpen extends InstantCommand {
  public LAPGOpen() {
    super();
    requires(Robot.m_lapg);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_lapg.open();
  }

}
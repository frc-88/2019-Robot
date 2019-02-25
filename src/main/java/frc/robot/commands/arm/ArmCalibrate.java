/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * ArmCalibrate
 * 
 * This command is used to calibrate the arm.
 * First, pose the arm so that both joints are
 * pointed straight up. Hold that position and
 * run the ArmCalibrate command.
 */
public class ArmCalibrate extends InstantCommand {
  /**
   * Add your docs here.
   */
  public ArmCalibrate() {
    super();
    requires(Robot.m_arm);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.m_arm.calibrate();
  }

}

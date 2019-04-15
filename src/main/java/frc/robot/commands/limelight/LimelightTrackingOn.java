/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class LimelightTrackingOn extends InstantCommand {
  /**
   * Add your docs here.
   */
  public LimelightTrackingOn() {
    super();
    requires(Robot.m_limelight);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.m_limelight.ledPipeline();
    Robot.m_limelight.camVision();
  }

}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class DriveStartRecording extends InstantCommand {
  /**
   * Add your docs here.
   */
  public DriveStartRecording() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_drive);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Preferences prefs = Preferences.getInstance();

    String filename = prefs.getString("Drive:RecordFile", "test");

    Robot.m_drive.startRecording(filename);
  }

}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DrivePlayback extends Command {
  private List<List<String>> points = new ArrayList<>();

  public DrivePlayback(String filename) {
    requires(Robot.m_drive);

    try {
      private BufferedReader br = new BufferedReader(new FileReader(new File(filename)));

      String line;
      while ((line = br.readLine()) != null) {
        String[] values = line.split(",");
        points.add(Arrays.asList(values));
      }

      br.close();
    } catch (IOException ioe) {
      ioe.printStackTrace();
    }
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import frc.robot.Robot;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.sql.Timestamp;
import java.text.SimpleDateFormat;

import edu.wpi.first.wpilibj.command.Command;

public class IntakeTester extends Command {
  private static final SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMddHHmmss");

  private double speed = -1.0;
  private FileWriter fw;
  private BufferedWriter bw;

  public IntakeTester() {
    requires(Robot.m_intake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    try {
      fw = new FileWriter(String.format("/home/lvuser/intake/%s.log",sdf.format(new Timestamp(System.currentTimeMillis()))));
      bw = new BufferedWriter(fw);
    } catch (IOException ioe) {
      ioe.printStackTrace();
    }

    Robot.m_intake.set(speed);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_intake.logDistanceData(bw);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_intake.set(0);
    try {
      bw.close();
    } catch (IOException ioe) {
      ioe.printStackTrace();
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

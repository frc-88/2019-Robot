/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive.test;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class DriveIncrease extends Command {

    private final double VOLTAGE_INC = 0.0001;
    private final double SECONDS_PER_LOOP = 0.02;
    private final int DIRECTION_CHANGE = (int)Math.round(2. / SECONDS_PER_LOOP);


    private double curVoltage = 0;
    private double direction = 1;

    private int loopCount = 0;

  public DriveIncrease() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
     curVoltage = 0;
     direction = 1;
     loopCount = 0;


  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double driveVoltage = curVoltage * direction;
    Robot.m_drive.basicDrive(driveVoltage, driveVoltage);
    curVoltage += VOLTAGE_INC;
    loopCount++;
    if (loopCount % DIRECTION_CHANGE == 0) {
        direction *= -1;
    }
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
    end();
  }
}

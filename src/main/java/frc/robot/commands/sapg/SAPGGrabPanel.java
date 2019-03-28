/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.sapg;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class SAPGGrabPanel extends Command {

    private int state;
    private long startTime;
    private final long PUSH_TIME=600000; // microseconds
    private final long OPEN_TIME=600000; // microseconds

  public SAPGGrabPanel() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_sapg);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
      state=0;
      startTime=RobotController.getFPGATime();

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      switch(state){
          case 0: 
          //close, push out
          Robot.m_sapg.close();
          Robot.m_sapg.deploy();
          if (RobotController.getFPGATime()-startTime>PUSH_TIME){
              state++;
              startTime=RobotController.getFPGATime();

          }
          break;
          case 1:
          //open, push out
          Robot.m_sapg.open();
          Robot.m_sapg.deploy();
          if (RobotController.getFPGATime()-startTime>OPEN_TIME){
            state++;
            startTime=RobotController.getFPGATime();
          }
          break;
          case 2:
          //open, pull in
          Robot.m_sapg.open();
          Robot.m_sapg.retract();
          if (RobotController.getFPGATime()-startTime>PUSH_TIME){
            state++;
            startTime=RobotController.getFPGATime();
          }
          break;

      }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return state==3;
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

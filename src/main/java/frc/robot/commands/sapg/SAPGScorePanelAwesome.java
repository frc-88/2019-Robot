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

public class SAPGScorePanelAwesome extends Command {

    private int state;
    private long startTime;
    private final long PUSH_TIME=500000; // microseconds
    private final long CLOSE_TIME=500000; // microseconds
    private final long END_TIME = 2000000; //microseconds

  public SAPGScorePanelAwesome() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_sapg);
    requires(Robot.m_drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
      state=0;
      startTime=RobotController.getFPGATime();
      Robot.m_drive.arcadeDrive(0, 0);
      Robot.m_sapg.trackingOff();

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double speed;
    double turn;

      switch(state){
          case 0: 
          //push out
          Robot.m_drive.arcadeDrive(0, 0);
          Robot.m_sapg.deploy();
          Robot.m_sapg.open();
          if (RobotController.getFPGATime()-startTime>PUSH_TIME){
              state++;
              startTime=RobotController.getFPGATime();

          }
          break;
          case 1:
          //open, push out
          Robot.m_drive.arcadeDrive(0, 0);
          Robot.m_sapg.close();
          if (RobotController.getFPGATime()-startTime>CLOSE_TIME){
            state++;
            startTime=RobotController.getFPGATime();
          }
          break;
          case 2:
          //open, pull in
          Robot.m_drive.arcadeDrive(0, 0);
          Robot.m_sapg.retract();
          if (RobotController.getFPGATime()-startTime>PUSH_TIME){
            state++;
            startTime=RobotController.getFPGATime();
          }
          break;

          case 3:
          //wait before centering takes over
          speed = Robot.m_oi.getDriverLeftYAxis();
          turn = Robot.m_oi.getDriverRightXAxis();
          Robot.m_drive.arcadeDrive(speed, turn);
          Robot.m_drive.autoshift();

          Robot.m_sapg.disable();

          if (RobotController.getFPGATime() - startTime > END_TIME) {
            state++;
            startTime = RobotController.getFPGATime();
          }

      }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return state==4;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_drive.arcadeDrive(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

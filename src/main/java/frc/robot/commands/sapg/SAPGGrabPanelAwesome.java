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

public class SAPGGrabPanelAwesome extends Command {

  private int state;
  private long startTime;
  private final long PUSH_TIME = 500000; // microseconds
  private final long CLOSE_TIME = 500000; // microseconds
  private final long END_TIME = 2000000; //microseconds
  private final double STOP_DISTANCE = 7; // inches

  public SAPGGrabPanelAwesome() {
    requires(Robot.m_drive);
    requires(Robot.m_sapg);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    state = 0;
    Robot.m_sapg.trackingOn();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double speed;
    double turn;

    switch (state) {
    case 0:
      // Driving and tracking
      speed = Robot.m_oi.getDriverLeftYAxis();
      turn = Robot.m_oi.getDriverRightXAxis();
      Robot.m_drive.arcadeDrive(speed, turn);
      Robot.m_drive.autoshift();


      if (Robot.m_sapg.isTracking() && Robot.m_limelight_sapg.getTargetDistance() < 96) {
      
        //double x = Robot.m_limelight_sapg.getTargetDistance() * Math.sin(Math.toRadians(Robot.m_limelight_sapg.getHorizontalOffsetAngle())) * -1;
        double x = 14 * Math.sin(Math.toRadians(Robot.m_limelight_sapg.getHorizontalOffsetAngle())) * -1;
  
  
        int targetPosition = 535 + (int) Math.round(x * 23);
  
 
        Robot.m_sapg.goToPosition(targetPosition);
      }

      if (Robot.m_sapg.getPanelDistance() < STOP_DISTANCE) {
        state++;
        startTime = RobotController.getFPGATime();
      }

      break;

    case 1:
      // push out
      Robot.m_drive.arcadeDrive(0, 0);
      Robot.m_sapg.trackingOff();
      Robot.m_sapg.deploy();
      Robot.m_sapg.close();
      if (RobotController.getFPGATime() - startTime > PUSH_TIME) {
        state++;
        startTime = RobotController.getFPGATime();

      }
      break;
    case 2:
      // open, push out
      Robot.m_drive.arcadeDrive(0, 0);
      Robot.m_sapg.open();
      if (RobotController.getFPGATime() - startTime > CLOSE_TIME) {
        state++;
        startTime = RobotController.getFPGATime();
      }
      break;
    case 3:
      // open, pull in
      Robot.m_drive.arcadeDrive(0, 0);
      Robot.m_sapg.retract();
      if (RobotController.getFPGATime() - startTime > PUSH_TIME) {
        state++;
        startTime = RobotController.getFPGATime();
      }
      break;

    case 4:
      //wait before centering takes over
      speed = Robot.m_oi.getDriverLeftYAxis();
      turn = Robot.m_oi.getDriverRightXAxis();
      Robot.m_drive.arcadeDrive(speed, turn);
      Robot.m_drive.autoshift();

      if (RobotController.getFPGATime() - startTime > END_TIME) {
        state++;
        startTime = RobotController.getFPGATime();
      }

    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return state == 5;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_drive.arcadeDrive(0, 0);
    Robot.m_sapg.trackingOff();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

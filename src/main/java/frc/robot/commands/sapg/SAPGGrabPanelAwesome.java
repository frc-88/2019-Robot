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
  private final double STOP_DISTANCE = 7; // inches

  public SAPGGrabPanelAwesome() {
    requires(Robot.m_drive);
    requires(Robot.m_sapg);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    state = 0;
    Robot.m_limelight_sapg.camVision();
    Robot.m_limelight_sapg.ledPipeline();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    System.out.println(state);
    switch (state) {
    case 0:
      // Driving and tracking
      double speed = Robot.m_oi.getDriverLeftYAxis();
      double turn = Robot.m_oi.getDriverRightXAxis();
      Robot.m_drive.arcadeDrive(speed, turn);
      Robot.m_drive.autoshift();

      Robot.m_sapg.enable();

      if (Robot.m_sapg.getPanelDistance() < STOP_DISTANCE) {
        state++;
        startTime = RobotController.getFPGATime();
      }

      break;

    case 1:
      // push out
      Robot.m_drive.arcadeDrive(0, 0);
      Robot.m_sapg.disable();
      Robot.m_sapg.forwardPush();
      Robot.m_sapg.closeTheJaws();
      if (RobotController.getFPGATime() - startTime > PUSH_TIME) {
        state++;
        startTime = RobotController.getFPGATime();

      }
      break;
    case 2:
      // open, push out
      Robot.m_drive.arcadeDrive(0, 0);
      Robot.m_sapg.disable();
      Robot.m_sapg.openTheJaws();
      if (RobotController.getFPGATime() - startTime > CLOSE_TIME) {
        state++;
        startTime = RobotController.getFPGATime();
      }
      break;
    case 3:
      // open, pull in
      Robot.m_drive.arcadeDrive(0, 0);
      Robot.m_sapg.disable();
      Robot.m_sapg.reversePush();
      if (RobotController.getFPGATime() - startTime > PUSH_TIME) {
        state++;
        startTime = RobotController.getFPGATime();
      }
      break;

    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return state == 4;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_drive.arcadeDrive(0, 0);
    Robot.m_sapg.disable();
    Robot.m_limelight_sapg.ledOff();
    Robot.m_limelight_sapg.camDriver();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

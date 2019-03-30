/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;

public class ClimberClimb extends Command {

  private Climber climber = Robot.m_climber;
  private Arm arm = Robot.m_arm;
  private Drive drive = Robot.m_drive;

  private final double LIFT_SHOULDER_START = 86;
  private final double LIFT_SHOULDER_END = 127;
  
  private final double LIFT_ELBOW_START = 176;
  private final double LIFT_ELBOW_END = 160;

  private final double PULL_ELBOW_TARGET = 180;

  private final double CLEAR_SHOULDER_TARGET = 86;

  private final int CLEAR_CLIMBER_TARGET = 27000;

  private int state;

  public ClimberClimb() {
    requires(climber);
    requires(arm);
    requires(drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    state = 0;

    drive.basicDrive(0, 0);

    arm.configureCoastMode();
    climber.configForShoulderPID();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    switch (state) {

    // Lift the robot
    case 0:

      arm.setShoulderVoltage(0);
      climber.moveShoulder(LIFT_SHOULDER_END);

      double shoulderPos = arm.getShoulderAbsDegrees();
      double shoulderPercentDone = Math.max(0, (shoulderPos - LIFT_SHOULDER_START) / (LIFT_SHOULDER_END - LIFT_SHOULDER_START));
      double elbowTotalDist = LIFT_ELBOW_END - LIFT_ELBOW_START;
      arm.moveElbowAbs(LIFT_ELBOW_START + elbowTotalDist * shoulderPercentDone);

      if (Math.abs(shoulderPos - LIFT_SHOULDER_END) < RobotMap.ARM_TOLERANCE) {
        state++;

        arm.configureBrakeMode();
      }
      
      break;

    case 1:

      arm.moveElbowAbs(PULL_ELBOW_TARGET);

      if (Math.abs(arm.getElbowAbsDegrees() - PULL_ELBOW_TARGET) < RobotMap.ARM_TOLERANCE) {
        state++;
      }

      break;

    case 2:

      arm.setShoulderVoltage(-0.05);
      climber.moveShoulder(CLEAR_SHOULDER_TARGET);

      if (Math.abs(arm.getShoulderAbsDegrees() - CLEAR_SHOULDER_TARGET) < RobotMap.ARM_TOLERANCE) {
        state++;
      }

      break;

    case 3:

      drive.arcadeDrive(.1, 0); 
      drive.autoshift(); 

      if (climber.onPlatform()) {
        state++;
      }
    
    case 4:

      drive.arcadeDrive(0, 0);
      drive.autoshift();
      
      climber.moveEncoder(CLEAR_CLIMBER_TARGET);
      arm.moveShoulder(CLEAR_SHOULDER_TARGET);
      arm.moveElbow(PULL_ELBOW_TARGET);

      break;

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
    climber.stop();
    arm.stopArm();
    drive.basicDrive(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

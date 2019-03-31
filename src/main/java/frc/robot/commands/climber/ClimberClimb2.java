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
import frc.robot.util.ArmPosition;

public class ClimberClimb2 extends Command {

  private Climber climber = Robot.m_climber;
  private Arm arm = Robot.m_arm;
  private Drive drive = Robot.m_drive;

  private final double LIFT_SHOULDER_START = 110;
  private final double LIFT_SHOULDER_END = 127;
  
  private final double LIFT_ELBOW_START = 165;
  private final double LIFT_ELBOW_END = 160;

  private final int LIFT_CLIMBER_TARGET = 38500;

  private final double PULL_ELBOW_TARGET = 174;

  private final double DROP_SHOULDER_TARGET = 121;
  private final int DROP_CLIMBER_TARGET = 36500;

  private final double CLEAR_SHOULDER_TARGET = 110;

  private final int CLEAR_CLIMBER_TARGET = 33000;

  private int state;

  public ClimberClimb2() {
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
    climber.configForEncoderPID();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    switch (state) {

    // Lift the robot
    case 0:

      arm.setShoulderVoltage(0);
      climber.moveEncoder(LIFT_CLIMBER_TARGET, 0.05);

      double shoulderPos = arm.getShoulderAbsDegrees();
      double shoulderPercentDone = Math.max(0, (shoulderPos - LIFT_SHOULDER_START) / (LIFT_SHOULDER_END - LIFT_SHOULDER_START));
      double elbowTotalDist = LIFT_ELBOW_END - LIFT_ELBOW_START;
      arm.moveElbowAbs(LIFT_ELBOW_START + elbowTotalDist * shoulderPercentDone);

      if (climber.targetReached()) {
        state++;

        arm.configureBrakeMode();

        arm.setElbowSpeed(50);
      }

      
      break;

    case 1:

      arm.moveElbowAbs(PULL_ELBOW_TARGET);
      drive.basicDrive(0.2, 0.2);

      if (Math.abs(arm.getElbowAbsDegrees() - PULL_ELBOW_TARGET) < RobotMap.ARM_TOLERANCE + 2) {
        state++;

        arm.setShoulderSpeed(50);        
      }

      break;

    case 2:

      climber.configForEncoderPID();
      climber.moveEncoder(DROP_CLIMBER_TARGET);

      arm.moveShoulder(DROP_SHOULDER_TARGET);

      drive.basicDrive(0.2, 0.2);

      if (climber.onPlatform()) {
        state = 4;
      }

      if (Math.abs(arm.getShoulderAbsDegrees() - DROP_SHOULDER_TARGET) < RobotMap.ARM_TOLERANCE) {
        state++;
      }

      break;

    case 3:

      drive.basicDrive(0.2, 0.2);

      if (climber.onPlatform()) {
        state++;
      }

      break;
    
    case 4:

      drive.basicDrive(0, 0);
      
      climber.moveEncoder(CLEAR_CLIMBER_TARGET);
      arm.moveShoulder(CLEAR_SHOULDER_TARGET);
      arm.moveElbow(PULL_ELBOW_TARGET);

      break;

    }
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return state == 10;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    climber.stop();
    arm.stopArm();
    drive.basicDrive(0, 0);
    arm.configureBrakeMode();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
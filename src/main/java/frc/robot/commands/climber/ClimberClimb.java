/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Preferences;
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
  private final double LIFT_SHOULDER_END = 122;
  
  private final double LIFT_ELBOW_START = 176;
  private final double LIFT_ELBOW_END = 160;

  private int LIFT_CLIMBER_TARGET = 48500;

  private final double PULL_ELBOW_TARGET = 173;

  private final double DROP_SHOULDER_TARGET = 121;
  private int DROP_CLIMBER_TARGET = 46500;

  private final double CLEAR_SHOULDER_TARGET = 86;

  private int CLEAR_CLIMBER_TARGET = 35000;

  private int state;
  private double leftDriveTarget;
  private double rightDriveTarget;
  private boolean leftDone;
  private boolean rightDone;

  public ClimberClimb() {
    requires(climber);
    requires(arm);
    requires(drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    state = 0;

    leftDone = false;
    rightDone = false;

    drive.basicDrive(0, 0);

    arm.configureCoastMode();
    climber.configForEncoderPID();

    Preferences prefs = Preferences.getInstance();
    if (prefs.containsKey("Climber:LiftTarget")) {
      LIFT_CLIMBER_TARGET = prefs.getInt("Climber:LiftTarget", LIFT_CLIMBER_TARGET);
    } else {
      prefs.putInt("Climber:LiftTarget", LIFT_CLIMBER_TARGET);
    }
    if (prefs.containsKey("Climber:DropTarget")) {
      DROP_CLIMBER_TARGET = prefs.getInt("Climber:DropTarget", DROP_CLIMBER_TARGET);
    } else {
      prefs.putInt("Climber:DropTarget", DROP_CLIMBER_TARGET);
    }
    if (prefs.containsKey("Climber:ClearTarget")) {
      CLEAR_CLIMBER_TARGET = prefs.getInt("Climber:ClearTarget", CLEAR_CLIMBER_TARGET);
    } else {
      prefs.putInt("Climber:ClearTarget", CLEAR_CLIMBER_TARGET);
    }
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
      double shoulderPercentDone = Math.min(Math.max(0, (shoulderPos - LIFT_SHOULDER_START) / (LIFT_SHOULDER_END - LIFT_SHOULDER_START)), 1);
      double elbowTotalDist = LIFT_ELBOW_END - LIFT_ELBOW_START;
      arm.moveElbowAbs(LIFT_ELBOW_START + elbowTotalDist * shoulderPercentDone, 0);

      if (climber.targetReached(3000) && Math.abs(arm.getElbowAbsDegrees() - LIFT_ELBOW_END) < RobotMap.ARM_TOLERANCE) {
        state++;
      }

      
      break;

    case 1:

      if (arm.getElbowAbsDegrees() < 170) {
        arm.moveElbowAbs(PULL_ELBOW_TARGET, 1);
      } else {
        arm.moveElbowAbs(PULL_ELBOW_TARGET, 0);
      }
      drive.basicDrive(0.4, 0.4);

      if (Math.abs(arm.getElbowAbsDegrees() - PULL_ELBOW_TARGET) < RobotMap.ARM_TOLERANCE + 1) {
        state++;

        arm.moveElbowAbs(PULL_ELBOW_TARGET, 0);

        arm.setShoulderSpeed(50);        
      }

      break;

    case 2:

      climber.configForEncoderPID();
      climber.moveEncoder(DROP_CLIMBER_TARGET);

      arm.moveShoulder(DROP_SHOULDER_TARGET);

      arm.moveElbowAbs(PULL_ELBOW_TARGET, 0);

      drive.basicDrive(0.4, 0.4);

      if (Robot.m_navx.getPitch() < -25) {
        state = 10;
      }

      if (climber.onPlatform()) {
        state++;

        leftDriveTarget = drive.getLeftPosition();
        rightDriveTarget = drive.getRightPosition();
      }

      if (Math.abs(arm.getShoulderAbsDegrees() - DROP_SHOULDER_TARGET) < RobotMap.ARM_TOLERANCE) {
        state++;
      }

      break;

    case 3:

      drive.basicDrive(0.4, 0.4);

      arm.moveElbowAbs(PULL_ELBOW_TARGET, 0);

      if (Robot.m_navx.getPitch() < -25) {
        state = 10;
      }

      if (climber.onPlatform()) {
        state++;

        leftDriveTarget = drive.getLeftPosition();
        rightDriveTarget = drive.getRightPosition();
      }

      break;
    
    case 4:

      arm.moveElbowAbs(PULL_ELBOW_TARGET, 0);

      if (!climber.onPlatform()) {
        leftDone = true;
        rightDone = true;
      }

      double leftVoltage;
      if (leftDone) {
        leftVoltage = 0;
      } else if (drive.getLeftSpeed() > 0) {
        leftDriveTarget = Math.min(leftDriveTarget, drive.getLeftPosition() - .5/12.);
        leftVoltage = -0.1;
      } else if (drive.getLeftPosition() > leftDriveTarget) {
        leftVoltage = -0.08;
      } else {
        leftDone = true;
        leftVoltage = 0;
      }

      double rightVoltage;
      if (rightDone) {
        rightVoltage = 0;
      } if (drive.getRightSpeed() > 0) {
        rightDriveTarget = Math.min(rightDriveTarget, drive.getRightPosition() - .5/12.);
        rightVoltage = -0.08;
      } else if (drive.getRightPosition() > rightDriveTarget) {
        rightVoltage = -0.06;
      } else {
        rightDone = true;
        rightVoltage = 0;
      }

      drive.basicDrive(leftVoltage, rightVoltage);
      
      climber.moveEncoder(CLEAR_CLIMBER_TARGET);
      arm.moveShoulder(CLEAR_SHOULDER_TARGET);
      arm.moveElbow(PULL_ELBOW_TARGET);

      if (Robot.m_navx.getPitch() < -25) {
        state = 10;
        
        arm.configureCoastMode();
        climber.configForEncoderPID();

      }

      if (leftDone && rightDone) {
        state++;
      }

      break;

    case 5:

      climber.moveEncoder(CLEAR_CLIMBER_TARGET);
      arm.moveShoulder(CLEAR_SHOULDER_TARGET);
      arm.moveElbow(PULL_ELBOW_TARGET);

      drive.basicDrive(0, 0);

      if (Robot.m_navx.getPitch() < -25) {
        state = 10;
        
        arm.configureCoastMode();
        climber.configForEncoderPID();

      }

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

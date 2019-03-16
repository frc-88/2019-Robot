/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.util.ArmPosition;

public class ClimberClimb extends Command {

  private final double CLIMBER_TARGET = -18;

  Climber climber;
  Arm arm;

  //int state;

  //double seekingSpeed = RobotMap.CLIMBER_SEEKING_SPEED;
  //double curSpeed;
  //double seekingRamp = RobotMap.CLIMBER_SEEKING_RAMP;

  public ClimberClimb() {
    requires(Robot.m_climber);
    requires(Robot.m_arm);

    climber = Robot.m_climber;
    arm = Robot.m_arm;

    SmartDashboard.putNumber("climber:seekingSpeed", RobotMap.CLIMBER_SEEKING_SPEED);
    SmartDashboard.putNumber("climber:seekingRamp", RobotMap.CLIMBER_SEEKING_RAMP);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // state = 0;

    // seekingSpeed = SmartDashboard.getNumber("climber:seekingSpeed", RobotMap.CLIMBER_SEEKING_SPEED);
    // seekingRamp = SmartDashboard.getNumber("climber:seekingRamp", RobotMap.CLIMBER_SEEKING_RAMP);
    // curSpeed = 0;

    // Robot.m_navx.zeroPitch();

    climber.zeroEncoder();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // switch (state) {

    // // Get the climber to the ground
    // case 0:

    //   arm.moveShoulder(SHOULDER_START_POS);
    //   arm.moveElbow(ELBOW_START_POS);
      
    //   curSpeed = Math.max(curSpeed + seekingRamp, seekingSpeed);
    //   climber.set(curSpeed);

    //   if (climber.isLifting()) {
    //     climber.stop();
    //     climber.zeroEncoder();
    //     state = 1;
    //   }
      
    //   break;

    // case 1:

      double shoulderStart = ArmPosition.CLIMB_PREP[0];
      double shoulderEnd = ArmPosition.CLIMB_LIFTED[0];
      double shoulderMaxSpeed = 40;
      double elbowStart = ArmPosition.CLIMB_PREP[1];
      double elbowEnd = ArmPosition.CLIMB_LIFTED[1];
      double elbowMaxSpeed = RobotMap.ELBOW_MAX_SPEED;
      double climberStart = 0;
      double climberEnd = CLIMBER_TARGET;
      double climberMaxSpeed = RobotMap.CLIMBER_MAX_SPEED;

      double shoulderDistance = Math.abs(shoulderEnd - shoulderStart);
      double elbowDistance = Math.abs(elbowEnd - elbowStart);
      double climberDistance = Math.abs(climberEnd - climberStart);

      double shoulderTime = shoulderDistance / shoulderMaxSpeed;
      double elbowTime = elbowDistance / elbowMaxSpeed;
      double climberTime = climberDistance / climberMaxSpeed;

      double time = Math.max(shoulderTime, Math.max(elbowTime, climberTime));

      arm.winchDown(shoulderEnd, shoulderDistance/time);

      arm.setElbowSpeed(elbowDistance/time);
      arm.moveElbow(elbowEnd);

      climber.move(climberEnd, climberDistance/time);

    //   if (climber.targetReached() && arm.targetReached()) {
    //     state = 2;
    //   }

    //   break;

    // }
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
    arm.setElbowSpeed(RobotMap.ELBOW_MAX_SPEED);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

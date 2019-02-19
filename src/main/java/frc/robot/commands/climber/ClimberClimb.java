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

public class ClimberClimb extends Command {

  private final double CLIMB_LIMIT = -20;
  private final double SHOULDER_START_POS = 90;
  private final double SHOULDER_END_POS = 129;
  private final double ELBOW_START_POS = 188;
  private final double ELBOW_END_POS = 172;

  Climber climber;
  Arm arm;

  int state;

  double seekingSpeed = RobotMap.CLIMBER_SEEKING_SPEED;
  double curSpeed;
  double seekingRamp = RobotMap.CLIMBER_SEEKING_RAMP;

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
    state = 0;

    seekingSpeed = SmartDashboard.getNumber("climber:seekingSpeed", RobotMap.CLIMBER_SEEKING_SPEED);
    seekingRamp = SmartDashboard.getNumber("climber:seekingRamp", RobotMap.CLIMBER_SEEKING_RAMP);
    curSpeed = 0;

    Robot.m_navx.zeroPitch();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    switch (state) {

    // Get the climber to the ground
    case 0:

      arm.moveShoulder(SHOULDER_START_POS);
      arm.moveElbow(ELBOW_START_POS);
      
      curSpeed = Math.max(curSpeed + seekingRamp, seekingSpeed);
      climber.set(curSpeed);

      if (climber.isLifting()) {
        climber.stop();
        state = 1;
      }
      
      break;

    case 1:

      climber.move(CLIMB_LIMIT);

      // (SHOULDER_START_POS-SHOULDER_END_POS)/(CLIMB_LIMIT/CLIMB_MAX_SPEED)
      arm.setShoulderSpeed((SHOULDER_START_POS-SHOULDER_END_POS)/(CLIMB_LIMIT/RobotMap.CLIMBER_MAX_SPEED));
      arm.setElbowSpeed((ELBOW_START_POS-ELBOW_END_POS)/(CLIMB_LIMIT/RobotMap.CLIMBER_MAX_SPEED)
      );

      arm.moveShoulder(SHOULDER_END_POS);
      arm.moveElbow(ELBOW_END_POS);

      break;

    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return state == 2;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    climber.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

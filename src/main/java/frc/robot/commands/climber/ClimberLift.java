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

public class ClimberLift extends Command {
  
  private Climber climber = Robot.m_climber;
  private Arm arm = Robot.m_arm;

  private final double SHOULDER_START = 86;
  private final double SHOULDER_END = 127;
  
  private final double ELBOW_START = 176;
  private final double ELBOW_END = 160;

  public ClimberLift() {
    requires(climber);
    requires(arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    arm.configureCoastMode();
    climber.configForShoulderPID();

    arm.setShoulderVoltage(0);
    climber.moveShoulder(SHOULDER_END);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double shoulderPos = arm.getShoulderAbsDegrees();
    double shoulderPercentDone = Math.max(0, (shoulderPos - SHOULDER_START) / (SHOULDER_END - SHOULDER_START));
    double elbowTotalDist = ELBOW_END - ELBOW_START;
    arm.moveElbowAbs(ELBOW_START + elbowTotalDist * shoulderPercentDone);
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    arm.configureBrakeMode();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeEjectCargo extends Command {
  private static final double INTAKE_SPEED = 1.0;

  private int counts;

  public IntakeEjectCargo() {
    requires(Robot.m_intake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    double shoulder = Robot.m_arm.getMotorShoulderDegrees();
    double elbow = Robot.m_arm.getMotorElbowDegrees();
    double speed = INTAKE_SPEED;

    counts = 0;

    if (shoulder > 0) {
      if (elbow > 90) speed = -speed;
    } else {
      if (elbow > -90) speed = -speed;
    }

    Robot.m_intake.set(speed);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return counts++ > 25;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_intake.set(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_intake.set(0.0);
  }
}

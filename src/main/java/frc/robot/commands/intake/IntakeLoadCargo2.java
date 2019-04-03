/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeLoadCargo2 extends Command {
  private double speed = 1.0;
  private double lastDistance;
  private double direction = -1.0;
  private int state;

  private int hasCounts;

  public IntakeLoadCargo2(double speed) {
    requires(Robot.m_intake);
    this.speed = Math.abs(speed);
    direction = Math.signum(speed);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    state = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double distance = Robot.m_intake.getAverageDistance();
    switch (state) {
    case 0:
      Robot.m_intake.set(speed * direction);
      hasCounts = 0;
      state++;
      break;

    case 1:
      if (Robot.m_intake.hasCargo()) {
        hasCounts++;
      }
      if (hasCounts > 0) {
        state++;
      }
      break;

    case 2:
      Robot.m_intake.set(0);
      speed = 0.5;
      state++;
      break;

    case 3:
      if (distance > lastDistance) {
        direction = direction * -1;
        speed -= 0.1;
      }
      Robot.m_intake.set(speed * direction);
      if (speed < 0.09) {
        state++;
      }
      break;

    }

    lastDistance = distance;

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return state == 4;
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

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
public class CargoPickupPosition extends Command {
  private static final double SHOULDER_POSITION=0;
  private static final double ELBOW_POSITION=0;

  public CargoPickupPosition() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
   Robot.m_arm.moveShoulder(SHOULDER_POSITION);
   Robot.m_arm.moveElbow(ELBOW_POSITION);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_arm.stopArm();
  }
}

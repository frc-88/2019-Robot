/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmGoToSetpoint;
import frc.robot.util.ArmPosition;

public class ClimberFullPrep2 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ClimberFullPrep2() {
    addParallel(new ArmGoToSetpoint(ArmPosition.PRE_CLIMB2));
    addSequential(new Command(){
    
      @Override
      protected boolean isFinished() {
        return Robot.m_arm.getCurrentSetpoint().equals(ArmPosition.PRE_CLIMB);
      }
    });
    addParallel(new ClimberPrep());
  }
}

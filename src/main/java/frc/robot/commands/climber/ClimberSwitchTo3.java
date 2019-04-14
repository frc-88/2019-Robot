/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.commands.ClimberSelectedCommand;
import frc.robot.commands.ClimberSelectedCommand.SelectState;
import frc.robot.commands.arm.ArmGoToSetpoint;
import frc.robot.util.ArmPosition;

public class ClimberSwitchTo3 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ClimberSwitchTo3() {
    addParallel(new ArmGoToSetpoint(ArmPosition.PRE_CLIMB));
    addSequential(new InstantCommand(){
    
      @Override
      protected void initialize() {
        ClimberSelectedCommand.selectState = SelectState.OFF;
      }
    });
    addParallel(new ClimberActivateLevel3());
  }
}

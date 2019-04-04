/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.arm.ArmGoToSetpoint;
import frc.robot.commands.intake.IntakeLoadCargo;
import frc.robot.util.ArmPosition;

public class ArmIntakeLoadCargo extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ArmIntakeLoadCargo() {
    addParallel(new IntakeLoadCargo(-1));
    addParallel(new ArmGoToSetpoint(ArmPosition.INTAKE));
  }
}

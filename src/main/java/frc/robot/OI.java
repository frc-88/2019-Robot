/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.util.TJController;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  private TJController driveController;
  private TJController operatorController;

  public OI() {
    operatorController= new TJController(RobotMap.OPERATOR_CONTROLLER_PORT);
    driveController = new TJController(RobotMap.DRIVE_CONTROLLER_PORT);
  }

  public double getDriverLeftXAxis() {
    return driveController.getLeftStickX();
  }

  public double getDriverLeftYAxis() {
    double rawValue = driveController.getLeftStickY();
    return Math.abs(rawValue) < .05 ? 0 : rawValue;
  }

  public double getDriverRightXAxis() {
    double rawValue = driveController.getRightStickX();
    return Math.abs(rawValue) < .05 ? 0 : rawValue;
  }

  public double getDriverRightYAxis() {
    double rawValue = driveController.getRightStickY();
    return Math.abs(rawValue) < .05 ? 0 : rawValue;
  }

  public boolean getHighGearButton(){
    return driveController.getRightTrigger()<.5;
  }

  public boolean getForceLowGearButton() {
    return driveController.getRightTrigger() > 0.5;
  }

  public double getOperatorLeftXAxis() {
    return operatorController.getLeftStickX();
  }

  public double getOperatorLeftYAxis() {
    return operatorController.getLeftStickY();
  }

  public double getOperatorRightXAxis() {
    return operatorController.getRightStickX();
  }

  public double getOperatorRightYAxis() {
    return operatorController.getRightStickY();
  }
}

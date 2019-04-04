/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.HaveCargoCommand;
import frc.robot.commands.arm.ArmBasicCommand;
import frc.robot.commands.arm.ArmCalibrate;
import frc.robot.commands.arm.ArmGoToPosition;
import frc.robot.commands.arm.ArmGoToPositionSafe;
import frc.robot.commands.arm.ArmGoToSetpoint;
import frc.robot.commands.arm.ArmZero;
import frc.robot.commands.climber.ClimberBasicControl;
import frc.robot.commands.climber.ClimberClimbChosen;
import frc.robot.commands.climber.ClimberDrop;
import frc.robot.commands.climber.ClimberFinish;
import frc.robot.commands.climber.ClimberLift;
import frc.robot.commands.climber.ClimberMoveEncoder;
import frc.robot.commands.climber.ClimberMoveShoulder;
import frc.robot.commands.climber.ClimberPrepChosen;
import frc.robot.commands.climber.ClimberPull;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.intake.IntakeBasicControl;
import frc.robot.commands.intake.IntakeDefault;
import frc.robot.commands.intake.IntakeEjectCargo;
import frc.robot.commands.intake.IntakeLoadCargo;
import frc.robot.commands.intake.IntakeManual;
import frc.robot.commands.navx.NavXZeroPitch;
import frc.robot.commands.navx.NavXZeroYaw;
import frc.robot.commands.sapg.SAPGClose;
import frc.robot.commands.sapg.SAPGDeploy;
import frc.robot.commands.sapg.SAPGOpen;
import frc.robot.commands.sapg.SAPGRetract;
import frc.robot.driveutil.DriveUtils;
import frc.robot.util.ArmPosition;
import frc.robot.util.TJButtonBox;
import frc.robot.util.TJController;
import frc.robot.commands.intake.IntakeTester;
import frc.robot.commands.limelight.LimelightTrackingOff;
import frc.robot.commands.limelight.LimelightTrackingOn;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  private TJController driveController;
  private TJController operatorController;
  private TJButtonBox buttonBox;

  public OI() {
    operatorController = new TJController(RobotMap.OPERATOR_CONTROLLER_PORT);
    driveController = new TJController(RobotMap.DRIVE_CONTROLLER_PORT);
    buttonBox = new TJButtonBox(RobotMap.BUTTON_BOX_PORT);

    // Driver controller
    driveController.buttonA.whenPressed(new LimelightTrackingOn());
    driveController.buttonA.whenReleased(new LimelightTrackingOff());
    driveController.buttonB.whenPressed(new LimelightTrackingOn());
    driveController.buttonB.whenReleased(new LimelightTrackingOff());
    driveController.buttonY.whenPressed(new ArcadeDrive());
    driveController.buttonBack.whenPressed(new ClimberClimbChosen());

    // Operator button box    
    buttonBox.buttonWhiteTopRight.whenPressed(new SAPGClose());
    buttonBox.buttonWhiteTopLeft.whenPressed(new SAPGOpen());
    buttonBox.buttonYellowTriangle.whenPressed(new IntakeManual(0.5));
    buttonBox.buttonYellowTriangle.whenReleased(new IntakeDefault());
    buttonBox.buttonRedLow.whenPressed(new ArmGoToSetpoint(ArmPosition.LOW_ROCKET));
    buttonBox.buttonBlueLow.whenPressed(new ArmGoToSetpoint(ArmPosition.LOW_ROCKET_BACK));
    buttonBox.buttonRedCargo.whenPressed(new ArmGoToSetpoint(ArmPosition.CARGO_SHIP_FRONT));
    buttonBox.buttonBlueCargo.whenPressed(new ArmGoToSetpoint(ArmPosition.CARGO_SHIP_BACK2));
    buttonBox.buttonRedMid.whenPressed(new ArmGoToSetpoint(ArmPosition.MEDIUM_ROCKET_FRONT));
    buttonBox.buttonBlueMid.whenPressed(new ArmGoToSetpoint(ArmPosition.MEDIUM_ROCKET_BACK2));
    buttonBox.buttonRedHigh.whenPressed(new ArmGoToSetpoint(ArmPosition.HIGH_ROCKET_FRONT));
    buttonBox.buttonBlueHigh.whenPressed(new ArmGoToSetpoint(ArmPosition.HIGH_ROCKET_BACK));
    buttonBox.buttonWhiteHome.whenPressed(new ArmGoToSetpoint(ArmPosition.HOME));
    buttonBox.buttonWhiteTriangle.whenPressed(new IntakeManual(-0.5));
    buttonBox.buttonWhiteTriangle.whenReleased(new IntakeDefault());
    buttonBox.buttonRedBig.whenPressed(new HaveCargoCommand(new IntakeEjectCargo(), new IntakeLoadCargo(-1)));
    buttonBox.buttonRedBig.whenPressed(new HaveCargoCommand(new InstantCommand(), new ArmGoToSetpoint(ArmPosition.INTAKE)));
    buttonBox.buttonWhiteRightTop.whenPressed(new SAPGRetract());
    buttonBox.buttonWhiteLeftTop.whenPressed(new SAPGDeploy());
    buttonBox.buttonGreenTriangle.whenPressed(new ClimberPrepChosen());

    // secondary testing controller
    switch (RobotMap.OPERATOR_CONTROL) {
    case RobotMap.OPERATOR_ARM_TEST:
      operatorController.buttonA.whenPressed(new ArmGoToSetpoint(ArmPosition.INTAKE));
      operatorController.buttonB.whenPressed(new ArmGoToSetpoint(ArmPosition.HOME);
      operatorController.buttonX.whenPressed(new ArmGoToSetpoint(ArmPosition.LOW_ROCKET));
      operatorController.buttonY.whenPressed(new ArmGoToSetpoint(ArmPosition.CARGO_SHIP_FRONT));
      operatorController.buttonRightBumper.whenPressed(new IntakeLoadCargo(-1));
      operatorController.buttonLeftBumper.whenPressed(new IntakeEjectCargo());
      break;

    case RobotMap.OPERATOR_SAPG_TEST:
      operatorController.buttonA.whenPressed(new SAPGDeploy());
      operatorController.buttonB.whenPressed(new SAPGRetract());
      operatorController.buttonX.whenPressed(new SAPGOpen());
      operatorController.buttonY.whenPressed(new SAPGClose());
      break;

    case RobotMap.OPERATOR_CLIMB_TEST:
      operatorController.buttonA.whenPressed(new ClimberLift());
      operatorController.buttonB.whenPressed(new ClimberPull());
      operatorController.buttonX.whenPressed(new ClimberDrop());
      operatorController.buttonY.whenPressed(new ClimberFinish());
      operatorController.buttonStart.whenPressed(new ClimberBasicControl());
      break;

    case RobotMap.OPERATOR_NONE:
    default:
      break;

    }

    // setup dashboard buttons for testing and debug
    SmartDashboard.putData("SAPG Deploy", new SAPGDeploy());
    SmartDashboard.putData("SAPG Retract", new SAPGRetract());
    SmartDashboard.putData("SAPG Open", new SAPGOpen());
    SmartDashboard.putData("SAPG Close", new SAPGClose());

    SmartDashboard.putData("Limelight On", new LimelightTrackingOn());
    SmartDashboard.putData("Limelight Off", new LimelightTrackingOff());

    SmartDashboard.putData("Arm Calibrate", new ArmCalibrate());
    SmartDashboard.putData("Arm Zero", new ArmZero());
    SmartDashboard.putData("Arm Goto Home", new ArmGoToSetpoint(ArmPosition.HOME));
    SmartDashboard.putData("Arm Goto Intake", new ArmGoToSetpoint(ArmPosition.INTAKE));
    SmartDashboard.putData("Arm Basic", new ArmBasicCommand());

    SmartDashboard.putData("Climb", new ClimberClimbChosen());
    SmartDashboard.putData("Climber Move Enc", new ClimberMoveEncoder());
    SmartDashboard.putData("Climber Move Shoulder", new ClimberMoveShoulder());
    SmartDashboard.putData("Climber Basic", new ClimberBasicControl());

    SmartDashboard.putData("Intake Cargo", new IntakeLoadCargo(-1));
    SmartDashboard.putData("Intake Eject", new IntakeEjectCargo());
    SmartDashboard.putData("Intake Tester", new IntakeTester());
    SmartDashboard.putData("Intake Basic", new IntakeBasicControl());

    SmartDashboard.putData("Zero Yaw", new NavXZeroYaw());
    SmartDashboard.putData("Zero Pitch", new NavXZeroPitch());

  }

  public double getDriverLeftXAxis() {
    return driveController.getLeftStickX();
  }

  public double getDriverLeftYAxis() {
    // double rawValue = driveController.getLeftStickY();
    // return Math.abs(rawValue) < .075 ? 0 : rawValue;
    return DriveUtils.deadbandExponential(driveController.getLeftStickY(), 1, .075);
  }

  public double getDriverRightXAxis() {
    // double rawValue = driveController.getRightStickX();
    // return Math.abs(rawValue) < .075 ? 0 : rawValue;
    return DriveUtils.deadbandExponential(driveController.getRightStickX(), 1, .075);
  }

  public double getDriverRightYAxis() {
    // double rawValue = driveController.getRightStickY();
    // return Math.abs(rawValue) < .075 ? 0 : rawValue;
    return DriveUtils.deadbandExponential(driveController.getRightStickY(), 1, .075);
  }

  public boolean getHighGearButton() {
    return driveController.getRightTrigger() > .5;
  }

  public boolean getForceLowGearButton() {
    return driveController.getRightTrigger() > 0.5;
  }

  public boolean isDriverButtonAPressed() {
    return driveController.getRawButton(1);
  }

  public boolean isDriverButtonBPressed() {
    return driveController.getRawButton(2);
  }

  public boolean isDriverButtonXPressed() {
    return driveController.getRawButton(3);
  }

  public boolean isDriverButtonYPressed() {
    return driveController.getRawButton(4);
  }

  public double getOperatorLeftXAxis() {
    // double rawValue = operatorController.getLeftStickX();
    // return Math.abs(rawValue) < .075 ? 0 : rawValue;
    return DriveUtils.deadbandExponential(operatorController.getLeftStickX(), 1, .075);
  }

  public double getOperatorLeftYAxis() {
    // double rawValue = operatorController.getLeftStickY();
    // return Math.abs(rawValue) < .075 ? 0 : rawValue;
    return DriveUtils.deadbandExponential(operatorController.getLeftStickY(), 1, .075);
  }

  public double getOperatorRightXAxis() {
    // double rawValue = operatorController.getRightStickX();
    // return Math.abs(rawValue) < .075 ? 0 : rawValue;
    return DriveUtils.deadbandExponential(operatorController.getRightStickX(), 1, .075);
  }

  public double getOperatorRightYAxis() {
    // double rawValue = operatorController.getRightStickY();
    // return Math.abs(rawValue) < .075 ? 0 : rawValue;
    return DriveUtils.deadbandExponential(operatorController.getRightStickY(), 1, .075);

  }

  public boolean getArmResetButton() {
    return buttonBox.buttonGreenTriangle.get();
  }
}

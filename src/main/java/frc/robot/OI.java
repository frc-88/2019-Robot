/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.HaveCargoCommand;
import frc.robot.commands.HavePanelCommand;
import frc.robot.commands.InPreClimbCommand;
import frc.robot.commands.arm.ArmBasicCommand;
import frc.robot.commands.arm.ArmCalibrate;
import frc.robot.commands.arm.ArmEStop;
import frc.robot.commands.arm.ArmGoToPosition;
import frc.robot.commands.arm.ArmGoToPositionSafe;
import frc.robot.commands.arm.ArmGoToSetpoint;
import frc.robot.commands.arm.ArmZero;
import frc.robot.commands.climber.ClimberBasicControl;
import frc.robot.commands.climber.ClimberClimb;
import frc.robot.commands.climber.ClimberDrop;
import frc.robot.commands.climber.ClimberFinish;
import frc.robot.commands.climber.ClimberFullPrep;
import frc.robot.commands.climber.ClimberLift;
import frc.robot.commands.climber.ClimberMoveEncoder;
import frc.robot.commands.climber.ClimberMoveShoulder;
import frc.robot.commands.climber.ClimberPrep;
import frc.robot.commands.climber.ClimberPull;
import frc.robot.commands.intake.IntakeBasicControl;
import frc.robot.commands.intake.IntakeDefault;
import frc.robot.commands.intake.IntakeEjectCargo;
import frc.robot.commands.intake.IntakeLoadCargo;
import frc.robot.commands.intake.IntakeManual;
import frc.robot.commands.navx.NavXZeroPitch;
import frc.robot.commands.navx.NavXZeroYaw;
import frc.robot.commands.sapg.SAPGClose;
import frc.robot.commands.sapg.SAPGDefault;
import frc.robot.commands.sapg.SAPGDeploy;
import frc.robot.commands.sapg.SAPGGrabPanel;
import frc.robot.commands.sapg.SAPGGrabPanelAwesome;
import frc.robot.commands.sapg.SAPGLoadPreferences;
import frc.robot.commands.sapg.SAPGOpen;
import frc.robot.commands.sapg.SAPGRetract;
import frc.robot.commands.sapg.SAPGScorePanel;
import frc.robot.commands.sapg.SAPGScorePanelAwesome;
import frc.robot.driveutil.DriveUtils;
import frc.robot.util.ArmPosition;
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
  private Joystick buttonBox;

  public OI() {
    operatorController = new TJController(RobotMap.OPERATOR_CONTROLLER_PORT);
    driveController = new TJController(RobotMap.DRIVE_CONTROLLER_PORT);
    buttonBox = new Joystick(RobotMap.BUTTON_BOX_PORT);

    driveController.buttonA.whenPressed(new LimelightTrackingOn());
    driveController.buttonA.whenReleased(new LimelightTrackingOff());

    new JoystickButton(buttonBox, 16).whenPressed(new SAPGClose());
    new JoystickButton(buttonBox, 14).whenPressed(new SAPGOpen());
    new JoystickButton(buttonBox, 12).whenPressed(new IntakeManual(0.5));
    new JoystickButton(buttonBox, 12).whenReleased(new IntakeDefault());
    new JoystickButton(buttonBox, 1).whenPressed(new ArmGoToSetpoint(ArmPosition.LOW_ROCKET));
    new JoystickButton(buttonBox, 5).whenPressed(new ArmGoToSetpoint(ArmPosition.LOW_ROCKET_BACK));
    new JoystickButton(buttonBox, 2).whenPressed(new ArmGoToSetpoint(ArmPosition.CARGO_SHIP_FRONT));
    new JoystickButton(buttonBox, 6).whenPressed(new ArmGoToSetpoint(ArmPosition.CARGO_SHIP_BACK2));
    new JoystickButton(buttonBox, 3).whenPressed(new ArmGoToSetpoint(ArmPosition.MEDIUM_ROCKET_FRONT));
    new JoystickButton(buttonBox, 7).whenPressed(new ArmGoToSetpoint(ArmPosition.MEDIUM_ROCKET_BACK2));
    new JoystickButton(buttonBox, 4).whenPressed(new ArmGoToSetpoint(ArmPosition.HIGH_ROCKET_FRONT));
    new JoystickButton(buttonBox, 8).whenPressed(new ArmGoToSetpoint(ArmPosition.HIGH_ROCKET_BACK));
    new JoystickButton(buttonBox, 9).whenPressed(new ArmGoToSetpoint(ArmPosition.HOME));
    new JoystickButton(buttonBox, 11).whenPressed(new IntakeManual(-0.5));
    new JoystickButton(buttonBox, 11).whenReleased(new IntakeDefault());
    new JoystickButton(buttonBox, 10).whenPressed(new HaveCargoCommand(new IntakeEjectCargo(), new IntakeLoadCargo(-1)));
    new JoystickButton(buttonBox, 10).whenPressed(new HaveCargoCommand(new InstantCommand(), new ArmGoToSetpoint(ArmPosition.INTAKE)));
    //new JoystickButton(buttonBox, 12).whenPressed(new SAPGGrabPanelAwesome());
    //new JoystickButton(buttonBox, 12).whenReleased(new SAPGRetract());
    new JoystickButton(buttonBox, 17).whenPressed(new SAPGRetract());
    //new JoystickButton(buttonBox, 11).whenPressed(new SAPGTrackStart());
    //new JoystickButton(buttonBox, 11).whenReleased(new SAPGScorePanelAwesome());
    new JoystickButton(buttonBox, 13).whenPressed(new SAPGDeploy());
    // new JoystickButton(buttonBox, 15).whenReleased(new ArmZeroShoulder());
    // new JoystickButton(buttonBox, 15).whenReleased(new ArmZeroElbow());
    new JoystickButton(buttonBox, 15).whenPressed(new InPreClimbCommand(new ClimberClimb(), new ClimberFullPrep()));

    switch (RobotMap.OPERATOR_CONTROL) {
    case RobotMap.OPERATOR_NONE:
      break;
    case RobotMap.OPERATOR_ARM_TEST:
      // intake
      operatorController.buttonA.whenPressed(new ArmGoToPosition(164, 85));
      // secure
      operatorController.buttonB.whenPressed(new ArmGoToPosition(160, 10));
      // low rocket
      operatorController.buttonX.whenPressed(new ArmGoToPosition(150, 0));
      // cargo ship
      operatorController.buttonY.whenPressed(new ArmGoToPosition(105, 35));
      operatorController.buttonRightBumper.whenPressed(new IntakeLoadCargo(-1));
      operatorController.buttonLeftBumper.whenPressed(new IntakeEjectCargo());
      break;
    case RobotMap.OPERATOR_SAPG_TEST:
      operatorController.buttonA.whenPressed(new SAPGDeploy());
      operatorController.buttonB.whenPressed(new SAPGRetract());
      operatorController.buttonX.whenPressed(new SAPGOpen());
      operatorController.buttonY.whenPressed(new SAPGClose());

      operatorController.buttonRightBumper.whenPressed(new SAPGGrabPanel());
      operatorController.buttonLeftBumper.whenPressed(new SAPGScorePanel());
      break;

    case RobotMap.OPERATOR_CLIMB_TEST:

      operatorController.buttonA.whenPressed(new ClimberLift());
      operatorController.buttonB.whenPressed(new ClimberPull());
      operatorController.buttonX.whenPressed(new ClimberDrop());
      operatorController.buttonY.whenPressed(new ClimberFinish());
      operatorController.buttonStart.whenPressed(new ClimberBasicControl());
      break;

    }

    // starting config
    //operatorController.buttonStart.whenPressed(new ArmGoToPosition(164, 0));

    // setup dashboard buttons for testing and debug
    //SmartDashboard.putData("Zero Yaw", new NavXZeroYaw());
    SmartDashboard.putData("Climber Basic", new ClimberBasicControl());

    SmartDashboard.putData("SAPG Deploy", new SAPGDeploy());
    SmartDashboard.putData("SAPG Retract", new SAPGRetract());
    SmartDashboard.putData("SAPG Open", new SAPGOpen());
    SmartDashboard.putData("SAPG Close", new SAPGClose());
    // SmartDashboard.putData("SAPG Grab", new SAPGGrabPanel());
    // SmartDashboard.putData("SAPG Score", new SAPGScorePanel());
    // SmartDashboard.putData("SAPG Grab Awesome", new SAPGGrabPanelAwesome());
    // SmartDashboard.putData("SAPG Score Awesome", new SAPGScorePanelAwesome());
    // SmartDashboard.putData("SAPG Load Prefs", new SAPGLoadPreferences());

    SmartDashboard.putData("Intake Tester", new IntakeTester());

    SmartDashboard.putData("Limelight On", new LimelightTrackingOn());
    SmartDashboard.putData("Limelight Off", new LimelightTrackingOff());

    SmartDashboard.putData("Arm Intake Position", new ArmGoToPosition(160, 82));
    SmartDashboard.putData("Arm Basic", new ArmBasicCommand());
    SmartDashboard.putData("Arm Calibrate", new ArmCalibrate());
    SmartDashboard.putData("Arm Zero", new ArmZero());
    SmartDashboard.putData("Arm Go To Position", new ArmGoToPosition());
    SmartDashboard.putData("Arm Go To Position Safe", new ArmGoToPositionSafe());
    // SmartDashboard.putData("Arm Start", new ArmGoToPositionSafe(ArmPosition.START));
    SmartDashboard.putData("Arm High Rocket", new ArmGoToPosition(28, 0));
    SmartDashboard.putData("Arm Cargo Ship",new ArmGoToPosition(105, 33));
    SmartDashboard.putData("Arm Starting Position", new ArmGoToPosition(150, 0));
    // SmartDashboard.putData("Arm Medium Rocket", new ArmGoToPositionSafe(ArmPosition.MEDIUM_ROCKET_FRONT));
    // SmartDashboard.putData("Arm Pre-Climb", new ArmGoToPositionSafe(ArmPosition.PRE_CLIMB));

    SmartDashboard.putData("Climb", new ClimberClimb());
    SmartDashboard.putData("Climber Move Enc", new ClimberMoveEncoder());
    SmartDashboard.putData("Climber Move Shoulder", new ClimberMoveShoulder());

    SmartDashboard.putData("Intake Basic", new IntakeBasicControl());
    SmartDashboard.putData("Intake Cargo", new IntakeLoadCargo(-1));
    SmartDashboard.putData("Intake Eject", new IntakeEjectCargo());

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
    return new JoystickButton(buttonBox, 15).get();
  }
}

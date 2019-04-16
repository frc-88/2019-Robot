/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArmInPositionCommand;
import frc.robot.commands.ArmIntakeLoadCargo;
import frc.robot.commands.ClimberSelectedCommand;
import frc.robot.commands.ClimberSwitchCommand;
import frc.robot.commands.HaveCargoCommand;
import frc.robot.commands.arm.ArmBasicCommand;
import frc.robot.commands.arm.ArmBattleMode;
import frc.robot.commands.arm.ArmCalibrate;
import frc.robot.commands.arm.ArmGoToSetpoint;
import frc.robot.commands.arm.ArmZero;
import frc.robot.commands.climber.ClimberActivateLevel3;
import frc.robot.commands.climber.ClimberBasicControl;
import frc.robot.commands.climber.ClimberClimb;
import frc.robot.commands.climber.ClimberClimb2;
import frc.robot.commands.climber.ClimberDrop;
import frc.robot.commands.climber.ClimberFinish;
import frc.robot.commands.climber.ClimberFullPrep;
import frc.robot.commands.climber.ClimberFullPrep2;
import frc.robot.commands.climber.ClimberHoldLevel2;
import frc.robot.commands.climber.ClimberLift;
import frc.robot.commands.climber.ClimberMoveEncoder;
import frc.robot.commands.climber.ClimberMoveShoulder;
import frc.robot.commands.climber.ClimberPrep;
import frc.robot.commands.climber.ClimberPrep2;
import frc.robot.commands.climber.ClimberPull;
import frc.robot.commands.climber.ClimberSwitchTo3;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.intake.IntakeBasicControl;
import frc.robot.commands.intake.IntakeDefault;
import frc.robot.commands.intake.IntakeEjectCargo;
import frc.robot.commands.intake.IntakeLoadCargo;
import frc.robot.commands.intake.IntakeLoadCargo2;
import frc.robot.commands.intake.IntakeManual;
import frc.robot.commands.navx.NavXZeroPitch;
import frc.robot.commands.navx.NavXZeroYaw;
import frc.robot.commands.lapg.LAPGActive;
import frc.robot.commands.lapg.LAPGClose;
import frc.robot.commands.lapg.LAPGDeploy;
import frc.robot.commands.lapg.LAPGGrab;
import frc.robot.commands.lapg.LAPGNeutral;
import frc.robot.commands.lapg.LAPGOpen;
import frc.robot.commands.lapg.LAPGRetract;
import frc.robot.commands.lapg.LAPGScore;
import frc.robot.driveutil.DriveUtils;
import frc.robot.util.ArmPosition;
import frc.robot.util.DebouncedButton;
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
  private TJController testController;
  private TJButtonBox buttonBox;

  private Trigger preclimbButton;

  public OI() {
    driveController = new TJController(RobotMap.DRIVE_CONTROLLER_PORT);
    buttonBox = new TJButtonBox(RobotMap.BUTTON_BOX_PORT);
    testController = new TJController(RobotMap.OPERATOR_CONTROLLER_PORT);

    // Driver controller
    driveController.buttonA.whenPressed(new LimelightTrackingOn());
    driveController.buttonA.whenReleased(new LimelightTrackingOff());
    driveController.buttonB.whenPressed(new LimelightTrackingOn());
    driveController.buttonB.whenReleased(new LimelightTrackingOff());
    driveController.buttonY.whenPressed(new ArcadeDrive());

    // Operator button box    
    buttonBox.buttonWhiteLeftTop.whenPressed(new LAPGDeploy());
    buttonBox.buttonWhiteTopLeft.whenPressed(new LAPGGrab());
    buttonBox.buttonWhiteTopRight.whenPressed(new LAPGScore());
    buttonBox.buttonWhiteRightTop.whenPressed(new LAPGRetract());
    buttonBox.buttonYellowTriangle.whenPressed(new IntakeManual(0.5));
    buttonBox.buttonYellowTriangle.whenReleased(new IntakeDefault());
    buttonBox.buttonWhiteTriangle.whenPressed(new IntakeManual(-0.5));
    buttonBox.buttonWhiteTriangle.whenReleased(new IntakeDefault());
    buttonBox.buttonGreenTriangle.whenPressed(new ArmZero());

    buttonBox.buttonRedLow.whenPressed(new ArmGoToSetpoint(ArmPosition.LOW_ROCKET));
    buttonBox.buttonRedCargo.whenPressed(new ArmGoToSetpoint(ArmPosition.CARGO_SHIP_FRONT));
    buttonBox.buttonBlueCargo.whenPressed(new ArmGoToSetpoint(ArmPosition.CARGO_SHIP_BACK2));
    buttonBox.buttonRedMid.whenPressed(new ArmGoToSetpoint(ArmPosition.MEDIUM_ROCKET_FRONT));
    buttonBox.buttonBlueMid.whenPressed(new ArmGoToSetpoint(ArmPosition.MEDIUM_ROCKET_BACK2));
    buttonBox.buttonRedHigh.whenPressed(new ArmGoToSetpoint(ArmPosition.HIGH_ROCKET_FRONT));
    buttonBox.buttonBlueHigh.whenPressed(new ArmGoToSetpoint(ArmPosition.HIGH_ROCKET_BACK));
    buttonBox.buttonWhiteHome.whenPressed(new ArmGoToSetpoint(ArmPosition.HOME));

    buttonBox.buttonBlueLow.whenPressed(new ArmInPositionCommand(ArmPosition.HOME, new ArmBattleMode()));
    buttonBox.buttonBlueLow.whenReleased(new ArmGoToSetpoint(ArmPosition.HOME));

    buttonBox.buttonRedBig.whenPressed(new HaveCargoCommand(new IntakeEjectCargo(), new ArmIntakeLoadCargo()));

    preclimbButton = new DebouncedButton(buttonBox.buttonBBBRed);
    preclimbButton.whenActive(new ClimberSwitchCommand(new ClimberFullPrep(), new ClimberFullPrep2()));
    buttonBox.buttonBBBBlue.whenPressed(new ClimberSelectedCommand(new ClimberClimb(), new ClimberClimb2()));
    buttonBox.buttonBBBSwitch.whenInactive(new ClimberSelectedCommand(new WaitCommand(0), new ClimberSwitchTo3()));

    // Backup Button Tab
    ShuffleboardTab backupTab = Shuffleboard.getTab("British Columbia");
    backupTab.add(new LAPGDeploy());
    backupTab.add(new LAPGRetract());
    backupTab.add(new LAPGGrab());
    backupTab.add(new LAPGScore());
    backupTab.add("Intake Forwards", new IntakeManual(0.5));
    backupTab.add("Intake Reverse", new IntakeManual(-0.5));
    backupTab.add(new ArmZero());
    backupTab.add("Red Button", new HaveCargoCommand(new IntakeEjectCargo(), new ArmIntakeLoadCargo()));

    backupTab.add("Low Rocket", new ArmGoToSetpoint(ArmPosition.LOW_ROCKET));
    backupTab.add("Cargo Ship", new ArmGoToSetpoint(ArmPosition.CARGO_SHIP_FRONT));
    backupTab.add("Cargo Ship Back", new ArmGoToSetpoint(ArmPosition.CARGO_SHIP_BACK2));
    backupTab.add("Mid Rocket", new ArmGoToSetpoint(ArmPosition.MEDIUM_ROCKET_FRONT));
    backupTab.add("Mid Rocket Back", new ArmGoToSetpoint(ArmPosition.MEDIUM_ROCKET_BACK2));
    backupTab.add("High Rocket", new ArmGoToSetpoint(ArmPosition.HIGH_ROCKET_FRONT));
    backupTab.add("High Rocket Back", new ArmGoToSetpoint(ArmPosition.HIGH_ROCKET_BACK));
    backupTab.add("Home", new ArmGoToSetpoint(ArmPosition.HOME));

    backupTab.add("Battle", new ArmBattleMode());

    backupTab.add("Preclimb 3", new ClimberFullPrep());
    backupTab.add("Preclimb 2", new ClimberFullPrep2());
    backupTab.add("Climb 3", new ClimberClimb());
    backupTab.add("Climb 2", new ClimberClimb2());

    // optional testing controller
    switch (RobotMap.OPERATOR_CONTROL) {
    case RobotMap.OPERATOR_ARM_TEST:
      testController.buttonA.whenPressed(new ArmGoToSetpoint(ArmPosition.INTAKE));
      testController.buttonB.whenPressed(new ArmGoToSetpoint(ArmPosition.HOME));
      testController.buttonX.whenPressed(new ArmGoToSetpoint(ArmPosition.LOW_ROCKET));
      testController.buttonY.whenPressed(new ArmGoToSetpoint(ArmPosition.CARGO_SHIP_FRONT));
      testController.buttonRightBumper.whenPressed(new IntakeLoadCargo(-1));
      testController.buttonLeftBumper.whenPressed(new IntakeEjectCargo());
      break;

    case RobotMap.OPERATOR_CLIMB_TEST:
      testController.buttonA.whenPressed(new ClimberLift());
      testController.buttonB.whenPressed(new ClimberPull());
      testController.buttonX.whenPressed(new ClimberDrop());
      testController.buttonY.whenPressed(new ClimberFinish());
      testController.buttonStart.whenPressed(new ClimberBasicControl());
      break;

    case RobotMap.OPERATOR_NONE:
    default:
      break;

    }

    // setup dashboard buttons for testing and debug
    SmartDashboard.putData("LAPG Deploy", new LAPGDeploy());
    SmartDashboard.putData("LAPG Retract", new LAPGRetract());
    SmartDashboard.putData("LAPG Open", new LAPGOpen());
    SmartDashboard.putData("LAPG Close", new LAPGClose());
    SmartDashboard.putData("LAPG Grab", new LAPGGrab());
    SmartDashboard.putData("LAPG Score", new LAPGScore());
    SmartDashboard.putData("LAPG Active", new LAPGActive());
    SmartDashboard.putData("LAPG Neutral", new LAPGNeutral());

    SmartDashboard.putData("Limelight On", new LimelightTrackingOn());
    SmartDashboard.putData("Limelight Off", new LimelightTrackingOff());

    SmartDashboard.putData("Arm Calibrate", new ArmCalibrate());
    SmartDashboard.putData("Arm Zero", new ArmZero());
    SmartDashboard.putData("Arm Goto Home", new ArmGoToSetpoint(ArmPosition.HOME));
    SmartDashboard.putData("Arm Goto Intake", new ArmGoToSetpoint(ArmPosition.INTAKE));
    SmartDashboard.putData("Arm Basic", new ArmBasicCommand());

    SmartDashboard.putData("Climber Move Enc", new ClimberMoveEncoder());
    SmartDashboard.putData("Climber Move Shoulder", new ClimberMoveShoulder());
    SmartDashboard.putData("Climber Basic", new ClimberBasicControl());
    SmartDashboard.putData("Climber Activate Level 3", new ClimberActivateLevel3());
    SmartDashboard.putData("Climber Hold Level 2", new ClimberHoldLevel2());

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

  public boolean getPushingModeButton() {
    return driveController.getLeftTrigger() > 0.5;
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
    return DriveUtils.deadbandExponential(testController.getLeftStickX(), 1, .075);
  }

  public double getOperatorLeftYAxis() {
    // double rawValue = operatorController.getLeftStickY();
    // return Math.abs(rawValue) < .075 ? 0 : rawValue;
    return DriveUtils.deadbandExponential(testController.getLeftStickY(), 1, .075);
  }

  public double getOperatorRightXAxis() {
    // double rawValue = operatorController.getRightStickX();
    // return Math.abs(rawValue) < .075 ? 0 : rawValue;
    return DriveUtils.deadbandExponential(testController.getRightStickX(), 1, .075);
  }

  public double getOperatorRightYAxis() {
    // double rawValue = operatorController.getRightStickY();
    // return Math.abs(rawValue) < .075 ? 0 : rawValue;
    return DriveUtils.deadbandExponential(testController.getRightStickY(), 1, .075);

  }

  public boolean getArmResetButton() {
    return buttonBox.buttonGreenTriangle.get();
  }

  public boolean inLevel3Mode() {
    return !buttonBox.buttonBBBSwitch.get();
  }
}

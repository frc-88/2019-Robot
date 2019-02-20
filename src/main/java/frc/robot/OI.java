/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.arm.ArmBasicCommand;
import frc.robot.commands.arm.ArmGoToPosition;
import frc.robot.commands.arm.ArmZeroElbow;
import frc.robot.commands.arm.ArmZeroShoulder;
import frc.robot.commands.climber.ClimberBasicControl;
import frc.robot.commands.climber.ClimberClimb;
import frc.robot.commands.climber.ClimberMove;
import frc.robot.commands.intake.IntakeBasicControl;
import frc.robot.commands.intake.IntakeEjectCargo;
import frc.robot.commands.intake.IntakeLoadCargo;
import frc.robot.commands.navx.NavXZeroPitch;
import frc.robot.commands.navx.NavXZeroYaw;
import frc.robot.commands.sapg.SAPGBasicControl;
import frc.robot.commands.sapg.SAPGClose;
import frc.robot.commands.sapg.SAPGDeploy;
import frc.robot.commands.sapg.SAPGGrabPanel;
import frc.robot.commands.sapg.SAPGOpen;
import frc.robot.commands.sapg.SAPGRetract;
import frc.robot.commands.sapg.SAPGScorePanel;
import frc.robot.commands.sapg.SAPGTrackStart;
import frc.robot.commands.sapg.SAPGTrackStop;
import frc.robot.commands.sapg.SAPGTrackTarget;
import frc.robot.driveutil.DriveUtils;
import frc.robot.util.TJController;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  private TJController driveController;
  private TJController operatorController;
  private Joystick buttonBox;

  public OI() {
    operatorController= new TJController(RobotMap.OPERATOR_CONTROLLER_PORT);
    driveController = new TJController(RobotMap.DRIVE_CONTROLLER_PORT);
    buttonBox = new Joystick(RobotMap.BUTTON_BOX_PORT);

    new JoystickButton(buttonBox, 16).whenPressed(new SAPGClose());
    new JoystickButton(buttonBox, 14).whenPressed(new SAPGOpen());
    new JoystickButton(buttonBox, 1).whenPressed(new ArmGoToPosition(150, 0));
    new JoystickButton(buttonBox, 5).whenPressed(new ArmGoToPosition(150, 0));
    new JoystickButton(buttonBox, 2).whenPressed(new ArmGoToPosition(105,35));
    new JoystickButton(buttonBox, 6).whenPressed(new ArmGoToPosition(105,35));
    new JoystickButton(buttonBox, 3).whenPressed(new ArmGoToPosition(80,0));
    new JoystickButton(buttonBox, 7).whenPressed(new ArmGoToPosition(80,0));
    new JoystickButton(buttonBox, 4).whenPressed(new ArmGoToPosition(28,0));
    new JoystickButton(buttonBox, 8).whenPressed(new ArmGoToPosition(28,0));
    new JoystickButton(buttonBox, 9).whenPressed(new ArmGoToPosition(160, 10));
    new JoystickButton(buttonBox, 13).whileHeld(new IntakeLoadCargo(1));
    
    new JoystickButton(buttonBox, 10).whenPressed(new ConditionalCommand(
      new IntakeEjectCargo(),
      new IntakeLoadCargo(-1)
    ) {

      @Override
      protected boolean condition() {
        return Robot.m_intake.hasCargo();
      }

    });
    new JoystickButton(buttonBox, 10).whenPressed(new ConditionalCommand(
      new ArmGoToPosition(164, 80)
    ) {

      @Override
      protected boolean condition() {
        return !Robot.m_intake.hasCargo();
      }

    });
    new JoystickButton(buttonBox, 12).whenPressed(new ConditionalCommand(
      new SAPGScorePanel(),
      new SAPGGrabPanel()
    ) {

      @Override
      protected boolean condition() {
        return Robot.m_sapg.hasPanel();
      }

    });

    if(false) {
    // intake position (164, 85)
    // secure position (160, 10)
    // low rocket  (150,0)
    // medium rocket  (87,0)
    // high rocket  (28,0)
    // cargo ship   (105,35)
    // starting config  (164,0)

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
    } else if (false) {
        //sapg controller
        operatorController.buttonA.whenPressed(new SAPGDeploy());
        operatorController.buttonB.whenPressed(new SAPGRetract());
        operatorController.buttonX.whenPressed(new SAPGOpen());
        operatorController.buttonY.whenPressed(new SAPGClose());


        operatorController.buttonRightBumper.whenPressed(new SAPGGrabPanel());
        operatorController.buttonLeftBumper.whenPressed(new SAPGScorePanel());
    }

      // starting config
      //operatorController.buttonStart.whenPressed(new ArmGoToPosition(164, 0));
    
    // setup dashboard buttons for testing and debug
    //SmartDashboard.putData("Zero Yaw", new NavXZeroYaw());
    SmartDashboard.putData("Climber Basic", new ClimberBasicControl());

    SmartDashboard.putData("SAPG Basic", new SAPGBasicControl());
    SmartDashboard.putData("SAPG Deploy", new SAPGDeploy());
    SmartDashboard.putData("SAPG Retract", new SAPGRetract());
    SmartDashboard.putData("SAPG Open", new SAPGOpen());
    SmartDashboard.putData("SAPG Close", new SAPGClose());
    SmartDashboard.putData("SAPG Grab", new SAPGGrabPanel());
    SmartDashboard.putData("SAPG Score", new SAPGScorePanel());
    SmartDashboard.putData("SAPG Track", new SAPGTrackTarget());
    SmartDashboard.putData("SAPG PID Go", new SAPGTrackStart());
    SmartDashboard.putData("SAPG PID Stop", new SAPGTrackStop());

    SmartDashboard.putData("Arm Basic", new ArmBasicCommand());
    SmartDashboard.putData("Arm Zero Elbow", new ArmZeroElbow());
    SmartDashboard.putData("Arm Zero Shoulder", new ArmZeroShoulder());
    SmartDashboard.putData("Arm Go To Position", new ArmGoToPosition());
    // high rocket  (28,0)
    SmartDashboard.putData("Arm High Rocket", new ArmGoToPosition(28,0));
    // medium rocket  (90,0)
    SmartDashboard.putData("Arm Medium Rocket", new ArmGoToPosition(90,0));

    SmartDashboard.putData("Climb", new ClimberClimb());
    SmartDashboard.putData("Move Climber", new ClimberMove());


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

  public boolean getHighGearButton(){
    return driveController.getRightTrigger()>.5;
  }

  public boolean getForceLowGearButton() {
    return driveController.getRightTrigger() > 0.5;
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
    return DriveUtils.deadbandExponential(driveController.getRightStickY(), 1, .075);
  }
}

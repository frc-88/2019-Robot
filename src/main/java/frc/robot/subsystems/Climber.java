/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  private final static int SLOTIDX = 0;
  private final static int TIMEOUTMS = 0;

  private TalonSRX winch;
  
  public Climber() {
      winch = new TalonSRX(RobotMap.CLIMBER_ID);
      configTalon(winch);
  }

  private void configTalon(TalonSRX talon) {
      talon.configFactoryDefault();
      talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, SLOTIDX, TIMEOUTMS);
      talon.configNominalOutputForward(0.0, TIMEOUTMS);
      talon.configNominalOutputReverse(0.0, TIMEOUTMS);
      talon.configPeakOutputForward(+1.0, TIMEOUTMS);
      talon.configClosedloopRamp(0, TIMEOUTMS);
      talon.configPeakOutputReverse(-1.0, TIMEOUTMS);
      talon.configNeutralDeadband(0.01, TIMEOUTMS);
      talon.setNeutralMode(NeutralMode.Brake);
      talon.config_kP(SLOTIDX, 0, TIMEOUTMS);
      talon.config_kI(SLOTIDX, 0, TIMEOUTMS);
      talon.config_kD(SLOTIDX, 0, TIMEOUTMS);
      talon.config_kF(SLOTIDX, 0, TIMEOUTMS);
      talon.configMotionCruiseVelocity(50, TIMEOUTMS);
      talon.configMotionAcceleration(50, TIMEOUTMS);
  }

  public void set(double percentOutput) {
    winch.set(ControlMode.PercentOutput, percentOutput);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

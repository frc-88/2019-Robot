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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.util.TJPIDController;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  private final static int ENCODER_PID_IDX = 0;
  private final static int TIMEOUTMS = 0;

  private TalonSRX winch;

  private double liftingCurrent = RobotMap.CLIMBER_LIFTING_CURRENT;
  
  public Climber() {
      winch = new TalonSRX(RobotMap.CLIMBER_ID);
      configTalon(winch);

      SmartDashboard.putNumber("climber:liftingCurrent", RobotMap.CLIMBER_LIFTING_CURRENT);
  }

  private void configTalon(TalonSRX talon) {
      talon.configFactoryDefault();
      talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, ENCODER_PID_IDX, TIMEOUTMS);
      talon.configNominalOutputForward(0.0, TIMEOUTMS);
      talon.configNominalOutputReverse(0.0, TIMEOUTMS);
      talon.configPeakOutputForward(+1.0, TIMEOUTMS);
      talon.configClosedloopRamp(0, TIMEOUTMS);
      talon.configPeakOutputReverse(-1.0, TIMEOUTMS);
      talon.configNeutralDeadband(0.01, TIMEOUTMS);
      talon.setNeutralMode(NeutralMode.Brake);
      talon.config_kP(ENCODER_PID_IDX, 1, TIMEOUTMS);
      talon.config_kI(ENCODER_PID_IDX, 0, TIMEOUTMS);
      talon.config_kD(ENCODER_PID_IDX, 0, TIMEOUTMS);
      talon.config_kF(ENCODER_PID_IDX, 0.06, TIMEOUTMS);
      talon.configMotionCruiseVelocity((RobotMap.CLIMBER_MAX_SPEED*RobotMap.CLIMBER_TICKS_PER_INCH)/10, TIMEOUTMS); // in/s -> ticks/100ms
      talon.configMotionAcceleration((2*RobotMap.CLIMBER_MAX_SPEED*RobotMap.CLIMBER_TICKS_PER_INCH)/10, TIMEOUTMS);
  }

  public void updateDashboard() {
    liftingCurrent = SmartDashboard.getNumber("climber:liftingCurrent", RobotMap.CLIMBER_LIFTING_CURRENT);
    SmartDashboard.putNumber("climber:currentDraw", winch.getOutputCurrent());
    SmartDashboard.putNumber("Climber: Position", getPosition());
    SmartDashboard.putNumber("Climber: Target", winch.getActiveTrajectoryPosition());
  }

  public void set(double percentOutput) {
    winch.set(ControlMode.PercentOutput, percentOutput);
  }

  public boolean isLifting() {
    return winch.getOutputCurrent() > liftingCurrent;
  }

  public void stop() {
    set(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void move(double distance){
    winch.set(ControlMode.MotionMagic, distance*RobotMap.CLIMBER_TICKS_PER_INCH);
  }

  public double getPosition(){
    double position = ((double)winch.getSelectedSensorPosition())/RobotMap.CLIMBER_TICKS_PER_INCH;
    return position;
  }
  public void zeroEncoder(){
    winch.setSelectedSensorPosition(0);
  }

  public boolean targetReached() {
    return Math.abs((winch.getClosedLoopError() * 1.)/RobotMap.CLIMBER_TICKS_PER_INCH) < RobotMap.CLIMBER_TOLERANCE;
  }
}

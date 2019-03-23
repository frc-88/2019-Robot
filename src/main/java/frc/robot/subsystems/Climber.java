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
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.util.SharpIR;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {

  private final static int ENCODER_PID_IDX = 0;
  private final static int SHOULDER_PID_IDX = 1;
  private final static int TIMEOUTMS = 0;

  private TalonSRX winch;

  private SharpIR platformIR;

  public Climber() {
      winch = new TalonSRX(RobotMap.CLIMBER_ID);
      configTalon(winch);

      platformIR = new SharpIR(RobotMap.CLIMBER_PLATFORM_IR_ID);
  }

  private void configTalon(TalonSRX talon) {
      talon.configFactoryDefault();

      talon.configNominalOutputForward(0.0, TIMEOUTMS);
      talon.configNominalOutputReverse(0.0, TIMEOUTMS);
      talon.configPeakOutputForward(+1.0, TIMEOUTMS);
      talon.configClosedloopRamp(0, TIMEOUTMS);
      talon.configPeakOutputReverse(-1.0, TIMEOUTMS);
      talon.configNeutralDeadband(0.01, TIMEOUTMS);
      talon.setNeutralMode(NeutralMode.Brake);

      talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, ENCODER_PID_IDX, TIMEOUTMS);
      talon.config_kP(ENCODER_PID_IDX, 1, TIMEOUTMS);
      talon.config_kI(ENCODER_PID_IDX, 0, TIMEOUTMS);
      talon.config_kD(ENCODER_PID_IDX, 0, TIMEOUTMS);
      
      talon.configRemoteFeedbackFilter(RobotMap.SHOULDER_AUXILARY_ID, RemoteSensorSource.TalonSRX_SelectedSensor, 0);
      talon.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, SHOULDER_PID_IDX, TIMEOUTMS);
      talon.config_kP(SHOULDER_PID_IDX, 1, TIMEOUTMS);
      talon.config_kI(SHOULDER_PID_IDX, 0, TIMEOUTMS);
      talon.config_kD(SHOULDER_PID_IDX, 1, TIMEOUTMS);
      talon.config_kF(SHOULDER_PID_IDX, 10, TIMEOUTMS);
      talon.configMotionCruiseVelocity(Robot.m_arm.convertShoulderDegreesToAbsCounts(RobotMap.CLIMB_ARM_SPEED)/10, TIMEOUTMS); // in/s -> ticks/100ms
      talon.configMotionAcceleration(3*Robot.m_arm.convertShoulderDegreesToAbsCounts(RobotMap.CLIMB_ARM_SPEED)/10, TIMEOUTMS);
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Climber: Position", getPosition());
    SmartDashboard.putNumber("Climber: Target", winch.getActiveTrajectoryPosition());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void stop() {
    setVoltage(0);
  }

  public void setVoltage(double percentOutput) {
    winch.set(ControlMode.PercentOutput, percentOutput);
  }

  public void moveEncoder(int distance){
    winch.selectProfileSlot(ENCODER_PID_IDX, ENCODER_PID_IDX);
    winch.set(ControlMode.MotionMagic, distance);
  }

  public void moveShoulder(double degrees) {
    winch.selectProfileSlot(SHOULDER_PID_IDX, SHOULDER_PID_IDX);
    winch.set(ControlMode.MotionMagic, Robot.m_arm.convertShoulderDegreesToAbsCounts(degrees));
  }

  public int getPosition(){
    return winch.getSelectedSensorPosition();
  }

  public void zeroEncoder(){
    winch.setSelectedSensorPosition(0);
  }

  public boolean targetReached() {
    return Math.abs((winch.getClosedLoopError())) < RobotMap.CLIMBER_TOLERANCE;
  }

  public boolean onPlatform() {
    return platformIR.getDistance() < RobotMap.PLATFORM_IR_THRESHOLD;
  }

}

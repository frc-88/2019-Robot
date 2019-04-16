/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.util.SharpIR;

/**
 * how did this happen?
 * there was no climber haiku
 * this explains it all
 */
public class Climber extends Subsystem {

  private final static int ENCODER_PID_IDX = 0;
  private final static int SHOULDER_PID_IDX = 1;
  private final static int TIMEOUTMS = 0;

  private TalonSRX winch;
  private TalonSRX follower;

  private SharpIR platformIR;
  private boolean prepped = false;

  private DoubleSolenoid selector;

  private int encoderTarget = 0;

  public Climber() {
      winch = new TalonSRX(RobotMap.CLIMBER_ID);
      follower = new TalonSRX(RobotMap.CLIMBER_FOLLOWER_ID);
      configTalon(winch);
      configFollower();

      platformIR = new SharpIR(RobotMap.CLIMBER_PLATFORM_IR_ID);
      prepped = false;

      selector = new DoubleSolenoid(RobotMap.CLIMBER_SELECTOR_PCM, 
          RobotMap.CLIMBER_SELECTOR_FORWARD, RobotMap.CLIMBER_SELECTOR_REVERSE);
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
      talon.setInverted(false);
      talon.setSensorPhase(true);

      talon.config_kP(ENCODER_PID_IDX, 2, TIMEOUTMS);
      talon.config_kI(ENCODER_PID_IDX, 0, TIMEOUTMS);
      talon.config_kD(ENCODER_PID_IDX, 0, TIMEOUTMS);
      
      talon.configRemoteFeedbackFilter(RobotMap.SHOULDER_ID, RemoteSensorSource.TalonSRX_SelectedSensor, 0);
      talon.config_kP(SHOULDER_PID_IDX, 2, TIMEOUTMS);
      talon.config_kI(SHOULDER_PID_IDX, 0, TIMEOUTMS);
      talon.config_kD(SHOULDER_PID_IDX, 0, TIMEOUTMS);
      talon.config_kF(SHOULDER_PID_IDX, 10, TIMEOUTMS);
      talon.configMotionCruiseVelocity(Robot.m_arm.convertShoulderDegreesToMotorCounts(RobotMap.CLIMB_ARM_SPEED)/10, TIMEOUTMS); // in/s -> ticks/100ms
      talon.configMotionAcceleration(3*Robot.m_arm.convertShoulderDegreesToMotorCounts(RobotMap.CLIMB_ARM_SPEED)/10, TIMEOUTMS);
  }

  private void configFollower() {
    follower.configFactoryDefault();
    follower.follow(winch);
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Climber: Position", getPosition());
    SmartDashboard.putNumber("Climber: SelectedSensorPos", winch.getSelectedSensorPosition());
    SmartDashboard.putNumber("Climber: Target", winch.getActiveTrajectoryPosition());
    SmartDashboard.putNumber("Climber: IR Distance", platformIR.getDistance());
  }

  public void configForEncoderPID() {
      winch.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, TIMEOUTMS);
      winch.selectProfileSlot(ENCODER_PID_IDX, 0);
      winch.setSensorPhase(true);
  }

  public void configForShoulderPID() {
      winch.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 0, TIMEOUTMS);
      winch.selectProfileSlot(SHOULDER_PID_IDX, 0);
      winch.setSensorPhase(true);
  }

  @Override
  public void initDefaultCommand() {
    // None
  }

  public void stop() {
    setVoltage(0);
  }

  public void prep() {
    prepped = true;
  }

  public boolean isPrepped() {
    return prepped;
  }

  public void setVoltage(double percentOutput) {
    winch.set(ControlMode.PercentOutput, percentOutput);
  }

  public void moveEncoder(int distance) {
    moveEncoder(distance, 0);
  }

  public void moveEncoder(int distance, double ff){
    winch.set(ControlMode.Position, distance,
        DemandType.ArbitraryFeedForward, ff);
    encoderTarget = distance;
  }

  public void moveShoulder(double degrees) {
    winch.set(ControlMode.MotionMagic, Robot.m_arm.convertShoulderDegreesToMotorCounts(degrees),
        DemandType.ArbitraryFeedForward, 0.1);
  }

  public int getPosition(){
    return winch.getSelectedSensorPosition(0);
  }

  public void zeroEncoder(){
    winch.setSelectedSensorPosition(0);
  }

  public boolean targetReached() {
    return targetReached(RobotMap.CLIMBER_TOLERANCE);
  }

  public boolean targetReached(int tolerance) {
    return Math.abs((getPosition() - encoderTarget)) < tolerance;
  }

  public boolean onPlatform() {
    return platformIR.getDistance() < RobotMap.PLATFORM_IR_THRESHOLD;
  }

  public void holdLevel2() {
    selector.set(Value.kForward);
  }

  public void activateLevel3() {
    selector.set(Value.kReverse);
  }

}

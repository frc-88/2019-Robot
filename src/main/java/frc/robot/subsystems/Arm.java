/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.*;

/**
 * Arm with a shoulder and an elbow joint, a relative encoder and absolute encoder on both.
 */

public class Arm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  TalonSRX shoulder, elbow;
  private final static int SLOTIDX = 0;
  private final static int TIMEOUTMS = 0;
  CANifier shoulderCan, elbowCan;	// CANifier


  public Arm() {
    shoulder = new TalonSRX(RobotMap.SHOULDER_ID);
    elbow = new TalonSRX(RobotMap.ELBOW_ID);
    shoulderCan=new CANifier(RobotMap.SHOULDER_CANIFIER_ID);
    elbowCan=new CANifier(RobotMap.ELBOW_CANIFIER_ID);

    configShoulderTalon();
    configElbowTalon();
    configCanifierCommon(shoulderCan);
    configCanifierCommon(elbowCan);
  }

  private void configShoulderCanifier() {
    configCanifierCommon(shoulderCan);
  }

  private void configElbowCanifier() {
    configCanifierCommon(elbowCan);
  }

  private void configShoulderTalon() {
    configTalonCommon(shoulder);
    shoulder.config_kP(SLOTIDX, 0, TIMEOUTMS);
    shoulder.config_kI(SLOTIDX, 0, TIMEOUTMS);
    shoulder.config_kD(SLOTIDX, 0, TIMEOUTMS);
    shoulder.config_kF(SLOTIDX, 0, TIMEOUTMS);
    shoulder.configMotionCruiseVelocity(50, TIMEOUTMS);
    shoulder.configMotionAcceleration(50, TIMEOUTMS);
  }

  private void configElbowTalon() {
    configTalonCommon(elbow);
    elbow.config_kP(SLOTIDX, 0, TIMEOUTMS);
    elbow.config_kI(SLOTIDX, 0, TIMEOUTMS);
    elbow.config_kD(SLOTIDX, 0, TIMEOUTMS);
    elbow.config_kF(SLOTIDX, 0, TIMEOUTMS);
    elbow.configMotionCruiseVelocity(50, TIMEOUTMS);
    elbow.configMotionAcceleration(50, TIMEOUTMS);
  }

  private void configTalonCommon(TalonSRX talon) {
    talon.configFactoryDefault();
    /* analog signal with no wrap-around (0-3.3V) */
    talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, SLOTIDX, TIMEOUTMS);

    talon.configNominalOutputForward(0.0, TIMEOUTMS);
    talon.configNominalOutputReverse(0.0, TIMEOUTMS);
    talon.configPeakOutputForward(+1.0, TIMEOUTMS);
    talon.configClosedloopRamp(0, TIMEOUTMS);
    talon.configPeakOutputReverse(-1.0, TIMEOUTMS);
    talon.configNeutralDeadband(0.01, TIMEOUTMS);
    talon.setNeutralMode(NeutralMode.Brake);

    /* eFeedbackNotContinuous = 1, subValue/ordinal/timeoutMs = 0 */

  }
  private void configCanifierCommon(CANifier canifier){
    canifier.configFactoryDefault();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

  }

  public void moveShoulder(double position) {
    shoulder.set(ControlMode.MotionMagic, position);
  }

  public void moveElbow(double position) {
    elbow.set(ControlMode.MotionMagic, position);
  }

  public void stopArm() {
    // stops the movement of the arm
  }

  public double getShoulderPosition() {
    return shoulder.getSelectedSensorPosition();
  }

  public double getElbowPosition() {
    return elbow.getSelectedSensorPosition();
  }
  
public double getShoulderAbsolutePosition(){
  return shoulderCan.getQuadraturePosition();
}

public double getElbowAbsolutePosition(){
  return elbowCan.getQuadraturePosition();
}

  /**
   * zeroes elbow motor encoder based on known elbow angle 
   * @param elbowAngle
   */
  public void zeroElbowMotorEncoder(double elbowAngle){
    // TODO: apply scaling facor to encoder position
    int encoderPosition=(int)elbowAngle;
    elbow.setSelectedSensorPosition(encoderPosition,0,TIMEOUTMS);
  }
    /**
   * zeroes shoulder motor encoder based on known shoulder angle 
   * @param shoulderAngle the angle of the shoulder
   */
  public void zeroShoulderMotorEncoder(double shoulderAngle){
    // TODO: apply scaling facor to encoder position
    int encoderPosition=(int)shoulderAngle;
    shoulder.setSelectedSensorPosition(encoderPosition,0,TIMEOUTMS);
  }
}

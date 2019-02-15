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

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.*;

/**
 * Arm with a shoulder and an elbow joint, a relative encoder and absolute encoder on both.
 */

public class Arm extends Subsystem {
  // TODO - check below constants against reality
  private final static double SHOULDER_LENGTH = 30.5;
  private final static double ELBOW_LENGTH = 24;
  private final static double ARM_HEIGHT = 41;
  private final static double ROBOT_LENGTH = 30;
  private final static double LEGAL_HEIGHT_LIMIT = 78;
  private final static double LEGAL_EXTENSION = 30;
  private final static double LEGAL_REACH = ROBOT_LENGTH / 2 + LEGAL_EXTENSION;
  private final static double FORWARD_SHOULDER_LIMIT = 170;
  private final static double REVERSE_SHOULDER_LIMIT = -60;
  private final static double FORWARD_ELBOW_LIMIT = 170;
  private final static double REVERSE_ELBOW_LIMIT = -170;
  private final static double ROBOT_TOP_LIMIT = -10;
  private final static double ROBOT_FORWARD_LIMIT = 5;
  private final static double ROBOT_REVERSE_LIMIT = 15;

  private final static int SLOTIDX = 0;
  private final static int TIMEOUTMS = 0;

  TalonSRX shoulder, elbow;
  Encoder shoulderEncoder, elbowEncoder;

  public Arm() {
    shoulder = new TalonSRX(RobotMap.SHOULDER_ID);
    elbow = new TalonSRX(RobotMap.ELBOW_ID);
    shoulderEncoder=new Encoder(RobotMap.SHOULDER_ENCODER_A,RobotMap.SHOULDER_ENCODER_B, false, Encoder.EncodingType.k4X);
    elbowEncoder=new Encoder(RobotMap.ELBOW_ENCODER_A,RobotMap.ELBOW_ENCODER_B, false, Encoder.EncodingType.k4X);

    configShoulderTalon();
    configElbowTalon();
    configShoulderEncoder();
    configElbowEncoder();

    initPreferences();
  }

  private void configShoulderEncoder() {
    configEncoderCommon(shoulderEncoder);
  }

  private void configElbowEncoder() {
    configEncoderCommon(elbowEncoder);
  }

  private void configShoulderTalon() {
    configTalonCommon(shoulder);
    // TODO determine these config values
    shoulder.config_kP(SLOTIDX, 0, TIMEOUTMS);
    shoulder.config_kI(SLOTIDX, 0, TIMEOUTMS);
    shoulder.config_kD(SLOTIDX, 0, TIMEOUTMS);
    shoulder.config_kF(SLOTIDX, 0, TIMEOUTMS);
    shoulder.configMotionCruiseVelocity(50, TIMEOUTMS);
    shoulder.configMotionAcceleration(50, TIMEOUTMS);
  }

  private void configElbowTalon() {
    configTalonCommon(elbow);
    // TODO determine these config values
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
  private void configEncoderCommon(Encoder encoder){
    // TODO 
    // write this
    //  see https://wpilib.screenstepslive.com/s/currentCS/m/java/l/599717-encoders-measuring-rotation-of-a-wheel-or-other-shaft
    //
    // for example:
    // encoder.setMaxPeriod(.1);
    // encoder.setMinRate(10);
    // encoder.setDistancePerPulse(5);
    // encoder.setReverseDirection(true);
    // encoder.setSamplesToAverage(7);
  }

  private void initPreferences() {
    Preferences prefs = Preferences.getInstance();

    if (!prefs.containsKey("ArmShoulderTarget")) { prefs.putDouble("ArmShoulderTarget", 0.0); }
    if (!prefs.containsKey("ArmElbowTarget")) { prefs.putDouble("ArmElbowTarget",0.0); }
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
  return shoulderEncoder.get();
}

public double getElbowAbsolutePosition(){
  return elbowEncoder.get();
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

  /**
   * isLegalPosition
   * 
   * args:
   *   targetShoulderAngle  the target angle for the shoulder
   *   targetElbowAngle     the target angle for the elbow
   *   inHabZone            set to true when you want to enforce
   *                        Hab zone height limits
   * 
   * returns: 
   *   boolean value indicating whether specified target position is legal
   */
  public boolean isLegalPosition(double targetShoulderAngle, double targetElbowAngle, boolean inHabZone) {
    double shoulderX = SHOULDER_LENGTH * Math.sin(Math.toRadians(targetShoulderAngle));
    double shoulderY = SHOULDER_LENGTH * Math.cos(Math.toRadians(targetShoulderAngle));
    double elbowX = ELBOW_LENGTH * Math.sin(Math.toRadians(targetElbowAngle)) + shoulderX;
    double elbowY = ELBOW_LENGTH * Math.cos(Math.toRadians(targetElbowAngle)) + shoulderY;
    boolean isLegal = true;

    // angles should be within range
    isLegal &= targetShoulderAngle < FORWARD_SHOULDER_LIMIT;
    isLegal &= targetShoulderAngle > REVERSE_SHOULDER_LIMIT;
    isLegal &= targetElbowAngle-targetShoulderAngle < FORWARD_ELBOW_LIMIT;
    isLegal &= targetElbowAngle-targetShoulderAngle > REVERSE_ELBOW_LIMIT; 

    // both points must be above the ground
    isLegal &= shoulderY > 0-ARM_HEIGHT;
    isLegal &= elbowY > 0-ARM_HEIGHT;

    // both points must not be within the robot
    isLegal &= shoulderY > ROBOT_TOP_LIMIT || shoulderX > ROBOT_FORWARD_LIMIT || shoulderX < ROBOT_REVERSE_LIMIT;
    isLegal &= elbowY > ROBOT_TOP_LIMIT || elbowX > ROBOT_FORWARD_LIMIT || elbowX < ROBOT_REVERSE_LIMIT;

    // both points must not go beyond our legal reach
    isLegal &= Math.abs(shoulderX) < LEGAL_REACH;
    isLegal &= Math.abs(elbowX) < LEGAL_REACH;

    // both points must be below the height limit in certain conditions
    isLegal &= inHabZone && shoulderY < LEGAL_HEIGHT_LIMIT-ARM_HEIGHT;
    isLegal &= inHabZone && elbowY < LEGAL_HEIGHT_LIMIT-ARM_HEIGHT;

    return isLegal;
  }

}

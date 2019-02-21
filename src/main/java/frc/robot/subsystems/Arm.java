/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.util.TJPIDController;

/**
 * Arm with a shoulder and an elbow joint, a relative encoder and absolute encoder on both.
 */

public class Arm extends Subsystem {
  // units in inches
  private final static double SHOULDER_LENGTH = 31;
  private final static double ELBOW_LENGTH = 24;
  private final static double ARM_HEIGHT = 41;
  private final static double ROBOT_LENGTH = 31.25;
  private final static double LEGAL_HEIGHT_LIMIT = 78;
  private final static double LEGAL_EXTENSION = 30;
  private final static double LEGAL_REACH = ROBOT_LENGTH / 2 + LEGAL_EXTENSION;
  private final static double PLATFORM_HEIGHT = 3;
  // TODO - check below constants against reality, above have been checked
  private final static double ROBOT_TOP_LIMIT = 0;
  private final static double ROBOT_FORWARD_LIMIT = 5;
  private final static double ROBOT_REVERSE_LIMIT = 15;
  private final static double FORWARD_SHOULDER_LIMIT = 170;
  private final static double REVERSE_SHOULDER_LIMIT = -60;
  private final static double FORWARD_ELBOW_LIMIT = 170;
  private final static double REVERSE_ELBOW_LIMIT = -170;

  private final static int MAIN_SLOT_IDX = 0;
  private final static int AUX_SENSOR_SLOT_IDX = 1;
  private final static int TIMEOUTMS = 0;


  private TalonSRX shoulder, elbow;
  private TJPIDController pitchPID;
  private int shoulderOffset = -1640;
  private int elbowOffset = 3665;


  public Arm() {
    shoulder = new TalonSRX(RobotMap.SHOULDER_ID);
    elbow = new TalonSRX(RobotMap.ELBOW_ID);

    configShoulderTalon();
    configElbowTalon();

    pitchPID = new TJPIDController(0.01, 0, 0);
    pitchPID.setTolerance(2);
    SmartDashboard.putNumber("arm:pitchKP", pitchPID.getKP());
    SmartDashboard.putNumber("arm:pitchKD", pitchPID.getKD());

    initPreferences();
  }

  private void configShoulderTalon() {
    configTalonCommon(shoulder);
    shoulder.config_kP(MAIN_SLOT_IDX, 4, TIMEOUTMS);
    shoulder.config_kI(MAIN_SLOT_IDX, 0, TIMEOUTMS);
    shoulder.config_kD(MAIN_SLOT_IDX, 0, TIMEOUTMS);
    shoulder.config_kF(MAIN_SLOT_IDX, 1.5, TIMEOUTMS);
    shoulder.configMotionCruiseVelocity(RobotMap.SHOULDER_MAX_SPEED * 4096 * 4 / 360 / 10, TIMEOUTMS);
    shoulder.configMotionAcceleration(2 * RobotMap.SHOULDER_MAX_SPEED * 4096 * 4 / 360 / 10, TIMEOUTMS);
    shoulder.setInverted(true);
    shoulder.configRemoteFeedbackFilter(RobotMap.SHOULDER_AUXILARY_ID, RemoteSensorSource.TalonSRX_SelectedSensor, 0, TIMEOUTMS);
  }

  private void configElbowTalon() {
    configTalonCommon(elbow);
    elbow.config_kP(MAIN_SLOT_IDX, 8, TIMEOUTMS);
    elbow.config_kI(MAIN_SLOT_IDX, 0, TIMEOUTMS);
    elbow.config_kD(MAIN_SLOT_IDX, 0, TIMEOUTMS);
    elbow.config_kF(MAIN_SLOT_IDX, 3.0, TIMEOUTMS);
    elbow.configMotionCruiseVelocity(RobotMap.ELBOW_MAX_SPEED * 4096 * 4 / 360 / 10, TIMEOUTMS);
    elbow.configMotionAcceleration(2 * RobotMap.ELBOW_MAX_SPEED * 4096 * 4 / 360 / 10, TIMEOUTMS);
    elbow.setInverted(false);
    elbow.configRemoteFeedbackFilter(RobotMap.ELBOW_AUXILARY_ID, RemoteSensorSource.TalonSRX_SelectedSensor, 0, TIMEOUTMS);
  }

  private void configTalonCommon(TalonSRX talon) {
    talon.configFactoryDefault();
    /* analog signal with no wrap-around (0-3.3V) */
    talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, MAIN_SLOT_IDX, TIMEOUTMS);
    talon.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, AUX_SENSOR_SLOT_IDX, TIMEOUTMS);

    talon.configNominalOutputForward(0.0, TIMEOUTMS);
    talon.configNominalOutputReverse(0.0, TIMEOUTMS);
    talon.configPeakOutputForward(+1.0, TIMEOUTMS);
    talon.configClosedloopRamp(0, TIMEOUTMS);
    talon.configPeakOutputReverse(-1.0, TIMEOUTMS);
    talon.configNeutralDeadband(0.01, TIMEOUTMS);
    talon.setNeutralMode(NeutralMode.Brake);
  }

  private void initPreferences() {
    Preferences prefs = Preferences.getInstance();

    // calibration prefs: position arm straight up and use ArmCalibrate command
    if (!prefs.containsKey("Arm:ShoulderOffset")) { prefs.putDouble("Arm:ShoulderOffset", shoulderOffset); }
    if (!prefs.containsKey("Arm:ElbowOffset")) { prefs.putDouble("Arm:ElbowOffset", elbowOffset); }
    // used by ArmGoToPosition
    if (!prefs.containsKey("Arm:ShoulderTarget")) { prefs.putDouble("Arm:ShoulderTarget", 0.0); }
    if (!prefs.containsKey("Arm:ElbowTarget")) { prefs.putDouble("Arm:ElbowTarget", 0.0); }
  }

  public void calibrate() {
    Preferences prefs = Preferences.getInstance();

    // set the current position to 0 degress and updates preferences
    shoulderOffset = getShoulderAbsolutePosition();
    elbowOffset = getElbowAbsolutePosition();

    prefs.putDouble("Arm:ShoulderOffset", shoulderOffset);
    prefs.putDouble("Arm:ElbowOffset", elbowOffset);
  }

  @Override
  public void initDefaultCommand() {
    // No default command
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Arm:shoulderPos", getShoulderPosition());
    SmartDashboard.putNumber("Arm:shoulderAbs", getShoulderAbsolutePosition());
    SmartDashboard.putNumber("Arm:elbowPos", getElbowPosition());
    SmartDashboard.putNumber("Arm:elbowAbs", getElbowAbsolutePosition());
    SmartDashboard.putNumber("Arm:shoulderDegrees", getShoulderDegrees());
    SmartDashboard.putNumber("Arm:elbowDegrees", getElbowDegrees());
    SmartDashboard.putNumber("Arm:shoulderMotorDegrees", getMotorShoulderDegrees());
    SmartDashboard.putNumber("Arm:elbowMotorDegrees", getMotorElbowDegrees());
    SmartDashboard.putNumber("Arm:shoulderSetPoint", shoulder.getActiveTrajectoryPosition());
    SmartDashboard.putNumber("Arm:elbowSetPoint", elbow.getActiveTrajectoryPosition());
    SmartDashboard.putBoolean("Arm:isSafe?", isSafePosition());
    SmartDashboard.putBoolean("Arm:isSafe(HAB)?", isSafePosition(true));

    pitchPID.setKP(SmartDashboard.getNumber("arm:pitchKP", pitchPID.getKP()));
    pitchPID.setKD(SmartDashboard.getNumber("arm:pitchKD", pitchPID.getKD()));
  }

  public void moveShoulder(double position) {
    shoulder.set(ControlMode.MotionMagic, position);
  }

  public void moveElbow(double position) {
    elbow.set(ControlMode.MotionMagic, position);
  }

  public void stopArm() {
    // stops the movement of the arm
    shoulder.set(ControlMode.PercentOutput, 0.0);
    elbow.set(ControlMode.PercentOutput, 0.0);
  }

  public void pidPitch(double pitch) {
    setShoulder(-pitchPID.calculateOutput(Robot.m_navx.getPitch(), pitch));
  }

  public double getShoulderPosition() {
    return shoulder.getSelectedSensorPosition(MAIN_SLOT_IDX);
  }

  public double getElbowPosition() {
    return elbow.getSelectedSensorPosition(MAIN_SLOT_IDX);
  }

  public int getShoulderAbsolutePosition() {

    return shoulder.getSelectedSensorPosition(AUX_SENSOR_SLOT_IDX);
  }

  public double convertShoulderToDegrees(double counts) {
    return ((counts - shoulderOffset) * 360) / 4096;
  }

  public double getShoulderDegrees() {

    return convertShoulderToDegrees(getShoulderAbsolutePosition());
  }

  public double convertMotorShoulderToDegrees(double counts) {
    return (((counts - shoulderOffset * 4) * 360) / 4096) / 4;
  }

  public int convertShoulderDegreesToMotor(double degrees) {
    return (int) (degrees * 4 * 4096 / 360 + shoulderOffset * 4);
  }

  public double getMotorShoulderDegrees() {
    return convertMotorShoulderToDegrees(getShoulderPosition());
  }

  public double convertElbowToDegrees(double counts) {
    return ((counts - elbowOffset) * 360) / 4096;
  }

  public double convertMotorElbowToDegrees(double counts) {
    return ((counts - elbowOffset * 4) * 360 / 4096) / 4;
  }

  public int convertElbowDegreesToMotor(double degrees) {
    return (int) degrees * 4 * 4096 / 360 + elbowOffset * 4;
  }

  public void setShoulderSpeed(int speed) {
    shoulder.configMotionCruiseVelocity(speed * 4096 * 4 / 360 / 10);
    shoulder.configMotionAcceleration(speed * 4096 * 4 / 360 / 10 * 2);
  }

  public void setElbowSpeed(int speed) {
    elbow.configMotionCruiseVelocity(speed * 4096 * 4 / 360 / 10);
    elbow.configMotionAcceleration(speed * 4096 * 4 / 360 / 10 * 2);
  }

  public double getMotorElbowDegrees() {
    return convertMotorElbowToDegrees(getElbowPosition());
  }

  public double getElbowDegrees() {
    return convertElbowToDegrees(getElbowAbsolutePosition());
  }

  public void setShoulder(double percentOutput) {
    shoulder.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setElbow(double percentOutput) {
    elbow.set(ControlMode.PercentOutput, percentOutput);
  }

  public int getElbowAbsolutePosition() {
    return elbow.getSelectedSensorPosition(AUX_SENSOR_SLOT_IDX);
  }

  /**
   * zeroes elbow motor encoder based on known elbow angle 
   * @param elbowAngle
   */
  public void zeroElbowMotorEncoder() {
    double auxEncoderPos = convertElbowToDegrees(getElbowAbsolutePosition());
    double normalizedPos = (auxEncoderPos + 180) > 0 ? 
        (auxEncoderPos + 180) % 360. - 180 : 
        (auxEncoderPos + 180) % 360. + 180;
    int encoderPos = convertElbowDegreesToMotor(normalizedPos);
    elbow.setSelectedSensorPosition(encoderPos, MAIN_SLOT_IDX, TIMEOUTMS);
  }

  /**
  * zeroes shoulder motor encoder based on known shoulder angle 
  * @param shoulderAngle the angle of the shoulder
  */
  public void zeroShoulderMotorEncoder() {
    double auxEncoderPos = convertShoulderToDegrees(getShoulderAbsolutePosition());
    double normalizedPos = auxEncoderPos > 0 ? 
        (auxEncoderPos + 180) % 360. - 180 : 
        (auxEncoderPos + 180) % 360. + 180;
    int encoderPos = convertShoulderDegreesToMotor(normalizedPos);
    shoulder.setSelectedSensorPosition(encoderPos, MAIN_SLOT_IDX, TIMEOUTMS);
  }

  /**
   * isSafePosition
   * 
   * args:
   *   targetShoulderAngle  the target angle for the shoulder
   *   targetElbowAngle     the target angle for the elbow
   *   inHabZone            set to true when you want to enforce
   *                        Hab zone height limits
   * 
   * returns: 
   *   boolean value indicating whether specified target position is safe and legal
   */
  public boolean isSafePosition() {
    // use current position, no HAB zone restrictions
    return isSafePosition(getShoulderDegrees(), getElbowDegrees(), false);
  }

  public boolean isSafePosition(boolean inHabZone) {
    // use current position
    return isSafePosition(getShoulderDegrees(), getElbowDegrees(), inHabZone);
  }

  public boolean isSafePosition(double targetShoulderAngle, double targetElbowAngle) {
    // default no HAB zone restrictions
    return isSafePosition(targetShoulderAngle, targetElbowAngle, inHabZone);
  }

  public boolean isSafePosition(double targetShoulderAngle, double targetElbowAngle, boolean inHabZone) {
    double shoulderX = SHOULDER_LENGTH * Math.sin(Math.toRadians(targetShoulderAngle));
    double shoulderY = SHOULDER_LENGTH * Math.cos(Math.toRadians(targetShoulderAngle));
    double elbowX = ELBOW_LENGTH * Math.sin(Math.toRadians(targetElbowAngle)) + shoulderX;
    double elbowY = ELBOW_LENGTH * Math.cos(Math.toRadians(targetElbowAngle)) + shoulderY;
    boolean isSafe = true;

    // both points must be above the ground
    isSafe &= shoulderY > 0 - ARM_HEIGHT;
    isSafe &= elbowY > 0 - ARM_HEIGHT;

    // both points must not be within the robot
    isSafe &= shoulderY > ROBOT_TOP_LIMIT || shoulderX > ROBOT_FORWARD_LIMIT || shoulderX < ROBOT_REVERSE_LIMIT;
    isSafe &= elbowY > ROBOT_TOP_LIMIT || elbowX > ROBOT_FORWARD_LIMIT || elbowX < ROBOT_REVERSE_LIMIT;

    // both points must not go beyond our legal reach
    isSafe &= Math.abs(shoulderX) < LEGAL_REACH;
    isSafe &= Math.abs(elbowX) < LEGAL_REACH;

    // both points must be below the height limit in certain conditions
    isSafe &= inHabZone && shoulderY < LEGAL_HEIGHT_LIMIT - ARM_HEIGHT - PLATFORM_HEIGHT;
    isSafe &= inHabZone && elbowY < LEGAL_HEIGHT_LIMIT - ARM_HEIGHT - PLATFORM_HEIGHT;

    // TODO angles should be within range
    // isSafe &= targetShoulderAngle < FORWARD_SHOULDER_LIMIT;
    // isSafe &= targetShoulderAngle > REVERSE_SHOULDER_LIMIT;
    // isSafe &= targetElbowAngle - targetShoulderAngle < FORWARD_ELBOW_LIMIT;
    // isSafe &= targetElbowAngle - targetShoulderAngle > REVERSE_ELBOW_LIMIT;
    
    return isSafe;
  }

}

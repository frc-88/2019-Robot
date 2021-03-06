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
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.util.ArmSetpoint;
import frc.robot.util.TJPIDController;

/**
 * 
 * a big beefy arm it can bend in two places elbow and shoulder
 * 
 */

public class Arm extends Subsystem {

  // Constants for arm safety checking, units are in inches
  private final static double SHOULDER_LENGTH = 31;
  private final static double ELBOW_LENGTH = 24;
  private final static double ARM_HEIGHT = 41;
  private final static double ROBOT_LENGTH = 31.25;
  private final static double LEGAL_HEIGHT_LIMIT = 78;
  private final static double LEGAL_EXTENSION = 30;
  private final static double LEGAL_REACH = ROBOT_LENGTH / 2 + LEGAL_EXTENSION;
  private final static double PLATFORM_HEIGHT = 3;
  private final static double CALEF_INCH = 1;
  // TODO - check below constants against reality, above have been checked
  private final static double ROBOT_TOP_LIMIT = 0;
  private final static double ROBOT_FORWARD_LIMIT = 5;
  private final static double ROBOT_REVERSE_LIMIT = 15;
  private final static double FORWARD_SHOULDER_LIMIT = 170;
  private final static double REVERSE_SHOULDER_LIMIT = -60;
  private final static double FORWARD_ELBOW_LIMIT = 170;
  private final static double REVERSE_ELBOW_LIMIT = -170;

  // Talon Info
  private final static int MAIN_SLOT_IDX = 0;
  private final static int AUX_SENSOR_SLOT_IDX = 1;
  private final static int MAGIC_PID_IDX = 0;
  private final static int POSITION_PID_IDX = 1;
  private final static int CURRENT_PID_IDX = 2;
  private final static int TIMEOUTMS = 0;

  private TalonSRX shoulder, elbow;

  // Offsets for absolute encoders
  private int shoulderOffset = -940;
  private int elbowOffset = 415;

  // Current arm setpoint
  private ArmSetpoint currentSetpoint = null;

  /****************************************************************************
   * INITIALIZATION AND CONFIGURATION
   ***************************************************************************/

  public Arm() {
    shoulder = new TalonSRX(RobotMap.SHOULDER_ID);
    elbow = new TalonSRX(RobotMap.ELBOW_ID);

    configShoulderTalon();
    configElbowTalon();

    initPreferences();
    fetchPreferences();
  }

  /**
   * Set the configuration parameters for the shoulder talon
   */
  private void configShoulderTalon() {
    configTalonCommon(shoulder);

    shoulder.config_kP(MAGIC_PID_IDX, 8, TIMEOUTMS);
    shoulder.config_kI(MAGIC_PID_IDX, 0, TIMEOUTMS);
    shoulder.config_kD(MAGIC_PID_IDX, 0, TIMEOUTMS);
    shoulder.config_kF(MAGIC_PID_IDX, 3, TIMEOUTMS);

    shoulder.config_kP(CURRENT_PID_IDX, 0.15, TIMEOUTMS);
    shoulder.config_kI(CURRENT_PID_IDX, 0, TIMEOUTMS);
    shoulder.config_kD(CURRENT_PID_IDX, 0,TIMEOUTMS);
    shoulder.config_kF(CURRENT_PID_IDX, .04, TIMEOUTMS);

    shoulder.setInverted(true);

    shoulder.configRemoteFeedbackFilter(RobotMap.SHOULDER_AUXILARY_ID, RemoteSensorSource.TalonSRX_SelectedSensor, 0,
        TIMEOUTMS);

    setShoulderSpeed(RobotMap.SHOULDER_MAX_SPEED);
  }

  /**
   * Set the configuration parameters for the elbow talon
   */
  private void configElbowTalon() {
    configTalonCommon(elbow);

    elbow.config_kP(MAGIC_PID_IDX, 8, TIMEOUTMS);
    elbow.config_kI(MAGIC_PID_IDX, 0, TIMEOUTMS);
    elbow.config_kD(MAGIC_PID_IDX, 0, TIMEOUTMS);
    elbow.config_kF(MAGIC_PID_IDX, 3.0, TIMEOUTMS);

    elbow.config_kP(POSITION_PID_IDX, 20, TIMEOUTMS);
    elbow.config_kI(POSITION_PID_IDX, 0, TIMEOUTMS);
    elbow.config_kD(POSITION_PID_IDX, 50, TIMEOUTMS);
    elbow.config_kF(POSITION_PID_IDX, 0, TIMEOUTMS);

    elbow.config_kP(CURRENT_PID_IDX, .15, TIMEOUTMS);
    elbow.config_kI(CURRENT_PID_IDX, 0, TIMEOUTMS);
    elbow.config_kD(CURRENT_PID_IDX, 0,TIMEOUTMS);
    elbow.config_kF(CURRENT_PID_IDX, .042, TIMEOUTMS);

    elbow.setInverted(false);

    elbow.configRemoteFeedbackFilter(RobotMap.ELBOW_AUXILARY_ID, RemoteSensorSource.TalonSRX_SelectedSensor, 0,
        TIMEOUTMS);

    setElbowSpeed(RobotMap.ELBOW_MAX_SPEED);
  }

  /**
   * Set the configuration parameters to the given talon that are common to both
   * the shoulder and elbow talons.
   */
  private void configTalonCommon(TalonSRX talon) {
    talon.configFactoryDefault();

    talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, MAIN_SLOT_IDX, TIMEOUTMS);
    talon.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, AUX_SENSOR_SLOT_IDX, TIMEOUTMS);

    talon.configNominalOutputForward(0.0, TIMEOUTMS);
    talon.configNominalOutputReverse(0.0, TIMEOUTMS);
    talon.configPeakOutputForward(+1.0, TIMEOUTMS);
    talon.configClosedloopRamp(0, TIMEOUTMS);
    talon.configPeakOutputReverse(-1.0, TIMEOUTMS);
    talon.configNeutralDeadband(0.01, TIMEOUTMS);
  }

  /**
   * Set both the shoulder and elbow to brake mode
   */
  public void configureBrakeMode() {
    shoulder.setNeutralMode(NeutralMode.Brake);
    elbow.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Set both the shoulder and elbow to coast mode
   */
  public void configureCoastMode() {
    shoulder.setNeutralMode(NeutralMode.Coast);
    elbow.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * Make sure that the preferences have all of the constants related to the arm
   * by setting defaults if they don't
   */
  private void initPreferences() {
    Preferences prefs = Preferences.getInstance();

    // calibration prefs: position arm straight up and use ArmCalibrate command
    if (!prefs.containsKey("Arm:ShoulderOffset")) {
      prefs.putDouble("Arm:ShoulderOffset", shoulderOffset);
    }
    if (!prefs.containsKey("Arm:ElbowOffset")) {
      prefs.putDouble("Arm:ElbowOffset", elbowOffset);
    }
    // used by ArmGoToPosition
    if (!prefs.containsKey("Arm:ShoulderTarget")) {
      prefs.putDouble("Arm:ShoulderTarget", 0.0);
    }
    if (!prefs.containsKey("Arm:ElbowTarget")) {
      prefs.putDouble("Arm:ElbowTarget", 0.0);
    }
    // arm positions adjustable by preference
    if (!prefs.containsKey("Arm:intake_shoulder")) {
      prefs.putDouble("Arm:intake_shoulder", 160);
    }
    if (!prefs.containsKey("Arm:intake_elbow")) {
      prefs.putDouble("Arm:intake_elbow", 82);
    }
  }

  /**
   * Pull all arm related constants from preferences into the code
   */
  private void fetchPreferences() {
    Preferences prefs = Preferences.getInstance();

    shoulderOffset = prefs.getInt("Arm:ShoulderOffset", shoulderOffset);
    elbowOffset = prefs.getInt("Arm:ElbowOffset", elbowOffset);
  }

  /**
   * Sets the shoulder and elbow offset so that both absolute encoders read 0
   * degrees.
   * 
   * To be clear: ONLY RUN THIS WHEN THE ARM IS POINTING STRAIGHT UPWARDS.
   */
  public void calibrateAbsoluteEncoders() {
    Preferences prefs = Preferences.getInstance();

    // set the current position to 0 degress and updates preferences
    shoulderOffset = getShoulderAbsCounts();
    elbowOffset = getElbowAbsCounts();

    prefs.putDouble("Arm:ShoulderOffset", shoulderOffset);
    prefs.putDouble("Arm:ElbowOffset", elbowOffset);

    zeroElbowMotorEncoder();
    zeroShoulderMotorEncoder();
  }

  /**
   * Updates all arm related values on the dashboard
   */
  public void updateDashboard() {
    SmartDashboard.putNumber("Arm:shoulderPos", getShoulderMotorCounts());
    SmartDashboard.putNumber("Arm:shoulderAbs", getShoulderAbsCounts());
    SmartDashboard.putNumber("Arm:elbowPos", getElbowMotorCounts());
    SmartDashboard.putNumber("Arm:elbowAbs", getElbowAbsCounts());
    SmartDashboard.putNumber("Arm:shoulderDegrees", getShoulderAbsDegrees());
    SmartDashboard.putNumber("Arm:elbowDegrees", getElbowAbsDegrees());
    SmartDashboard.putNumber("Arm:shoulderMotorDegrees", getShoulderMotorDegrees());
    SmartDashboard.putNumber("Arm:elbowMotorDegrees", getElbowMotorDegrees());
    SmartDashboard.putNumber("Arm:shoulderSetPoint",
        convertShoulderMotorCountsToDegrees(shoulder.getActiveTrajectoryPosition()));
    SmartDashboard.putNumber("Arm:elbowSetPoint",
        convertElbowMotorCountsToDegrees(elbow.getActiveTrajectoryPosition()));
    SmartDashboard.putBoolean("Arm:isSafe?", isSafePosition());
    SmartDashboard.putBoolean("Arm:isSafe(HAB)?", isSafePosition(true));
    SmartDashboard.putNumber("Arm:distFromBase", getDistanceFromBase());

    SmartDashboard.putNumber("Arm:shoulder offset", shoulderOffset);
    SmartDashboard.putNumber("Arm:elbow offset", elbowOffset);
    SmartDashboard.putNumber("Arm:shoulder current", shoulder.getOutputCurrent());
    SmartDashboard.putNumber("Arm:elbow current", elbow.getOutputCurrent());
  }

  /**
   * Zeros both encoders
   */
  public void zero() {
    zeroShoulderMotorEncoder();
    zeroElbowMotorEncoder();
  }

  /**
   * Zeroes shoulder motor encoder based on the shoulder absolute encoder
   */
  private void zeroShoulderMotorEncoder() {
    shoulder.setSelectedSensorPosition(convertShoulderDegreesToMotorCounts(getShoulderAbsDegrees()), MAIN_SLOT_IDX,
        TIMEOUTMS);
  }

  /**
   * Zeroes elbow motor encoder based on the elbow absolute encoder
   */
  private void zeroElbowMotorEncoder() {
    elbow.setSelectedSensorPosition(convertElbowDegreesToMotorCounts(getElbowAbsDegrees()), MAIN_SLOT_IDX, TIMEOUTMS);
  }

  @Override
  public void initDefaultCommand() {
    // No default command
  }

  /****************************************************************************
   * ENCODER CONVERSIONS
   ***************************************************************************/

  /**
   * Converts from shoulder absolute encoder counts to degrees.
   */
  public double convertShoulderAbsCountsToDegrees(int counts) {
    return ((counts - shoulderOffset) * 360.) / 4096.;
  }

  /**
   * Converts from shoulder degrees to absolute encoder counts.
   */
  public int convertShoulderDegreesToAbsCounts(double degrees) {
    return (int) ((degrees * 4096. / 360. + shoulderOffset));
  }

  /**
   * Converts from shoulder motor encoder counts to degrees.
   */
  public double convertShoulderMotorCountsToDegrees(double counts) {
    return (((counts - shoulderOffset * 4.) * 360.) / 4096.) / 4.;
  }

  /**
   * Converts from shoulder degrees to motor encoder counts.
   */
  public int convertShoulderDegreesToMotorCounts(double degrees) {
    return (int) (degrees * 4 * 4096 / 360 + shoulderOffset * 4);
  }

  /**
   * Converts from elbow absolute encoder counts to degrees.
   */
  public double convertElbowAbsCountsToDegrees(int counts) {
    return ((counts - elbowOffset) * 360.) / 4096.;
  }

  /**
   * Converts from elbow degrees to absolute encoder counts.
   */
  public int convertElbowDegreesToAbsCounts(double degrees) {
    return (int) (degrees * 4096 / 360 + elbowOffset);
  }

  /**
   * Converts from elbow motor encoder counts to degrees.
   */
  public double convertElbowMotorCountsToDegrees(double counts) {
    return (((counts - elbowOffset * 4.) * 360.) / 4096.) / 4.;
  }

  /**
   * Converts from elbow degrees to motor encoder counts.
   */
  public int convertElbowDegreesToMotorCounts(double degrees) {
    return (int) (degrees * 4 * 4096 / 360 + elbowOffset * 4);
  }

  /******************************************************************************
   * SENSOR GETTERS
   *****************************************************************************/

/**
   * Get the encoder counts read by the shoulder absolute encoder
   */
  public int getShoulderAbsCounts() {
    return shoulder.getSelectedSensorPosition(AUX_SENSOR_SLOT_IDX);
  }

  /**
   * Get the encoder counts read by the shoulder motor encoder
   */
  public int getShoulderMotorCounts() {
    return shoulder.getSelectedSensorPosition(MAIN_SLOT_IDX);
  }

  /**
   * Get the encoder counts read by the elbow absolute encoder
   */
  public int getElbowAbsCounts() {
    return elbow.getSelectedSensorPosition(AUX_SENSOR_SLOT_IDX);
  }

  /**
   * Get the encoder counts read by the shoulder motor encoder
   */
  public int getElbowMotorCounts() {
    return elbow.getSelectedSensorPosition(MAIN_SLOT_IDX);
  }

  /**
   * Get the shoulder degrees as read by the absolute encoder (after springs)
   */
  public double getShoulderAbsDegrees() {
    double encoderPos = convertShoulderAbsCountsToDegrees(getShoulderAbsCounts());
    double normalizedPos = (encoderPos + 180) > 0 ? (encoderPos + 180) % 360. - 180 : (encoderPos + 180) % 360. + 180;
    return normalizedPos;
  }

  /**
   * Get the shoulder degrees as read by the motor encoder (before springs)
   */
  public double getShoulderMotorDegrees() {
    return convertShoulderMotorCountsToDegrees(getShoulderMotorCounts());
  }

  /**
   * Get the elbow degrees as read by the absolute encoder (after springs)
   */
  public double getElbowAbsDegrees() {
    double encoderPos = convertElbowAbsCountsToDegrees(getElbowAbsCounts());
    double shoulderPos = getShoulderAbsDegrees();
    double diffFromShoulder = encoderPos - shoulderPos;
    double normalizedPos = (diffFromShoulder + 180) > 0 ? (diffFromShoulder + 180) % 360. - 180
        : (diffFromShoulder + 180) % 360. + 180;

    return normalizedPos + shoulderPos;
  }

  /**
   * Get the elbow degrees as read by the motor encoder (before springs)
   */
  public double getElbowMotorDegrees() {
    return convertElbowMotorCountsToDegrees(getElbowMotorCounts());
  }

  /**
   * Are both the shoulder and elbow PIDS within tolerance?
   */
  public boolean targetReached() {
    return Math.abs(convertShoulderMotorCountsToDegrees(shoulder.getClosedLoopError())) < RobotMap.ARM_TOLERANCE
        && Math.abs(convertElbowMotorCountsToDegrees(elbow.getClosedLoopError())) < RobotMap.ARM_TOLERANCE;
  }

  /******************************************************************************
   * MOVEMENT COMMANDS
   *****************************************************************************/

  /**
   * Gets the current setpoint for the arm.
   */
  public ArmSetpoint getCurrentSetpoint() {

    if (this.currentSetpoint == null) {
      return new ArmSetpoint(getShoulderMotorDegrees(), getElbowMotorDegrees());
    }

    return this.currentSetpoint;
  }

  /**
   * Sets the arm to move to the given setpoint with the given shoulder/elbow
   * targets in degrees and the given shoulder/elbow speeds in degrees/second
   */
  public void setSetpoint(ArmSetpoint to, 
      double shoulderTarget, double elbowTarget,
      double shoulderSpeed, double elbowSpeed) {

    this.currentSetpoint = to;

    setShoulderSpeed(shoulderSpeed);
    setElbowSpeed(elbowSpeed);

    shoulder.selectProfileSlot(MAGIC_PID_IDX, 0);
    elbow.selectProfileSlot(MAGIC_PID_IDX, 0);

    shoulder.set(ControlMode.MotionMagic, convertShoulderDegreesToMotorCounts(shoulderTarget));
    elbow.set(ControlMode.MotionMagic, convertElbowDegreesToMotorCounts(elbowTarget));
  }

  /**
   * Uses motion magic to send the shoulder to the given degrees setpoint
   */
  public void moveShoulder(double degrees) {
    currentSetpoint = null;
    shoulder.selectProfileSlot(MAGIC_PID_IDX, 0);
    shoulder.set(ControlMode.MotionMagic, convertShoulderDegreesToMotorCounts(degrees));
  }

  /**
   * Uses motion magic to send the elbow to the given degrees setpoint
   */
  public void moveElbow(double degrees) {
    currentSetpoint = null;
    elbow.selectProfileSlot(MAGIC_PID_IDX, 0);
    elbow.set(ControlMode.MotionMagic, convertElbowDegreesToMotorCounts(degrees));
  }

  /**
   * PIDs the elbow to the given position based off the absolute encoder which is
   * after the springs
   */
  public void moveElbowAbs(double degrees, double ff) {
    currentSetpoint = null;
    elbow.selectProfileSlot(POSITION_PID_IDX, 0);
    elbow.set(ControlMode.MotionMagic, convertElbowDegreesToMotorCounts(
        degrees + getElbowMotorDegrees() - getElbowAbsDegrees()),
        DemandType.ArbitraryFeedForward, ff);
  }

  /**
   * Sets the motion magic speed of the shoulder in degrees per second
   */
  public void setShoulderSpeed(double speed) {
    shoulder.configMotionCruiseVelocity((int) (speed * 4096. * 4. / 360. / 10.));
    shoulder.configMotionAcceleration((int) (speed * 4096. * 4. / 360. / 10. * RobotMap.ACCELERATION_RATIO));
  }

  /**
   * Sets the motion magic speed of the elbow is degrees per second
   */
  public void setElbowSpeed(double speed) {
    elbow.configMotionCruiseVelocity((int) (speed * 4096. * 4. / 360. / 10.));
    elbow.configMotionAcceleration((int) (speed * 4096. * 4. / 360. / 10. * RobotMap.ACCELERATION_RATIO));
  }

  /**
   * Commands the arm to zero motor output
   */
  public void stopArm() {
    currentSetpoint = null;
    shoulder.set(ControlMode.PercentOutput, 0.0);
    elbow.set(ControlMode.PercentOutput, 0.0);
  }

  /**
   * Sets the shoulder to the given percent output in open loop mode
   */
  public void setShoulderVoltage(double percentOutput) {
    currentSetpoint = null;
    shoulder.set(ControlMode.PercentOutput, percentOutput);
  }

  /**
   * Sets the elbow to the given percent output in open loop mode
   */
  public void setElbowVoltage(double percentOutput) {
    currentSetpoint = null;
    elbow.set(ControlMode.PercentOutput, percentOutput);
  }

  /**
   * PIDS the shoulder to the given current in Amps
   */
  public void setShoulderCurrent(double current) {
    shoulder.selectProfileSlot(CURRENT_PID_IDX, 0);
    shoulder.set(ControlMode.Current, current);
  }

  /**
   * PIDS the elbow to the given current in Amps
   */
  public void setElbowCurrent(double current) {
    elbow.selectProfileSlot(CURRENT_PID_IDX, 0);
    elbow.set(ControlMode.Current, current);
  }

  /*****************************************************************************
   * ARM SAFETY
   ****************************************************************************/

  /**
   * isSafePosition
   * 
   * args: targetShoulderAngle the target angle for the shoulder targetElbowAngle
   * the target angle for the elbow inHabZone set to true when you want to enforce
   * Hab zone height limits
   * 
   * returns: boolean value indicating whether specified target position is safe
   * and legal
   */
  public boolean isSafePosition() {
    // use current position, no HAB zone restrictions
    return isSafePosition(getShoulderAbsDegrees(), getElbowAbsDegrees(), false);
  }

  public boolean isSafePosition(boolean inHabZone) {
    // use current position
    return isSafePosition(getShoulderAbsDegrees(), getElbowAbsDegrees(), inHabZone);
  }

  public boolean isSafePosition(double targetShoulderAngle, double targetElbowAngle) {
    // default no HAB zone restrictions
    return isSafePosition(targetShoulderAngle, targetElbowAngle, false);
  }

  public boolean isSafePosition(double[] target) {
    // default no HAB zone restrictions
    return isSafePosition(target[0], target[1], false);
  }

  public boolean isSafePosition(double targetShoulderAngle, double targetElbowAngle, boolean inHabZone) {
    double shoulderX = SHOULDER_LENGTH * Math.sin(Math.toRadians(targetShoulderAngle));
    double shoulderY = SHOULDER_LENGTH * Math.cos(Math.toRadians(targetShoulderAngle));
    double elbowX = ELBOW_LENGTH * Math.sin(Math.toRadians(targetElbowAngle)) + shoulderX;
    double elbowY = ELBOW_LENGTH * Math.cos(Math.toRadians(targetElbowAngle)) + shoulderY;
    boolean isSafe = true;

    SmartDashboard.putNumber("Arm:shoulderX", shoulderX);
    SmartDashboard.putNumber("Arm:shoulderY", shoulderY);
    SmartDashboard.putNumber("Arm:elbowX", elbowX);
    SmartDashboard.putNumber("Arm:elbowY", elbowY);

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
    if (inHabZone) {
      isSafe &= shoulderY < LEGAL_HEIGHT_LIMIT - ARM_HEIGHT - PLATFORM_HEIGHT;
      isSafe &= elbowY < LEGAL_HEIGHT_LIMIT - ARM_HEIGHT - PLATFORM_HEIGHT;
    }

    // TODO angles should be within range
    // isSafe &= targetShoulderAngle < FORWARD_SHOULDER_LIMIT;
    // isSafe &= targetShoulderAngle > REVERSE_SHOULDER_LIMIT;
    // isSafe &= targetElbowAngle - targetShoulderAngle < FORWARD_ELBOW_LIMIT;
    // isSafe &= targetElbowAngle - targetShoulderAngle > REVERSE_ELBOW_LIMIT;

    return isSafe;
  }

  /*****************************************************************************
   * MISCELANEOUS
   ****************************************************************************/

  public double getDistanceFromBase() {
    double supportX = 0;
    double supportY = ARM_HEIGHT;
    double shoulderX = SHOULDER_LENGTH * Math.sin(Math.toRadians(getShoulderAbsDegrees()));
    double shoulderY = SHOULDER_LENGTH * Math.cos(Math.toRadians(getShoulderAbsDegrees()));
    double elbowX = ELBOW_LENGTH * Math.sin(Math.toRadians(getElbowAbsDegrees()));
    double elbowY = ELBOW_LENGTH * Math.cos(Math.toRadians(getElbowAbsDegrees()));

    double x1 = supportX + shoulderX + elbowX;
    double y1 = supportY + shoulderY + elbowY;

    double x2 = supportX + shoulderX;
    double y2 = supportY + shoulderY;

    return Math.max(Math.sqrt(x1 * x1 + y1 * y1), Math.sqrt(x2 * x2 + y2 * y2));

  }

  public boolean shoulderSkipped() {
    return Math.abs(getShoulderAbsDegrees() - getShoulderMotorDegrees()) > 45;
  }

  public boolean elbowSkipped() {
    return Math.abs(getShoulderAbsDegrees() - getElbowMotorDegrees()) > 45;
  }

}

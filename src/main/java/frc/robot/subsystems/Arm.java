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
 * 
 * a big beefy arm
 * it can bend in two places
 * elbow and shoulder
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
  private final static int TIMEOUTMS = 0;

  private TalonSRX shoulder, elbow;

  // Offsets for absolute encoders
  private int shoulderOffset = -940;
  private int elbowOffset = 415;

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
    
    shoulder.config_kP(MAIN_SLOT_IDX, 8, TIMEOUTMS);
    shoulder.config_kI(MAIN_SLOT_IDX, 0, TIMEOUTMS);
    shoulder.config_kD(MAIN_SLOT_IDX, 0, TIMEOUTMS);
    shoulder.config_kF(MAIN_SLOT_IDX, 3, TIMEOUTMS);
    
    shoulder.setInverted(true);

    shoulder.configRemoteFeedbackFilter(RobotMap.SHOULDER_AUXILARY_ID, RemoteSensorSource.TalonSRX_SelectedSensor, 0, TIMEOUTMS);

    setShoulderSpeed(RobotMap.SHOULDER_MAX_SPEED);
  }

  /**
   * Set the configuration parameters for the elbow talon
   */
  private void configElbowTalon() {
    configTalonCommon(elbow);

    elbow.config_kP(MAIN_SLOT_IDX, 8, TIMEOUTMS);
    elbow.config_kI(MAIN_SLOT_IDX, 0, TIMEOUTMS);
    elbow.config_kD(MAIN_SLOT_IDX, 0, TIMEOUTMS);
    elbow.config_kF(MAIN_SLOT_IDX, 3.0, TIMEOUTMS);

    elbow.setInverted(false);

    elbow.configRemoteFeedbackFilter(RobotMap.ELBOW_AUXILARY_ID, RemoteSensorSource.TalonSRX_SelectedSensor, 0, TIMEOUTMS);

    setElbowSpeed(RobotMap.ELBOW_MAX_SPEED);
  }

  /**
   * Set the configuration parameters to the given talon that are common to
   * both the shoulder and elbow talons.
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
   * Make sure that the preferences have all of the constants related to the
   * arm by setting defaults if they don't
   */
  private void initPreferences() {
    Preferences prefs = Preferences.getInstance();

    // calibration prefs: position arm straight up and use ArmCalibrate command
    if (!prefs.containsKey("Arm:ShoulderOffset")) { prefs.putDouble("Arm:ShoulderOffset", shoulderOffset); }
    if (!prefs.containsKey("Arm:ElbowOffset")) { prefs.putDouble("Arm:ElbowOffset", elbowOffset); }
    // used by ArmGoToPosition
    if (!prefs.containsKey("Arm:ShoulderTarget")) { prefs.putDouble("Arm:ShoulderTarget", 0.0); }
    if (!prefs.containsKey("Arm:ElbowTarget")) { prefs.putDouble("Arm:ElbowTarget", 0.0); }
    // arm positions adjustable by preference
    if (!prefs.containsKey("Arm:intake_shoulder")) { prefs.putDouble("Arm:intake_shoulder", 160); }
    if (!prefs.containsKey("Arm:intake_elbow")) { prefs.putDouble("Arm:intake_elbow", 82); }
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
    shoulderOffset = getShoulderAbsolutePosition();
    elbowOffset = getElbowAbsolutePosition();

    prefs.putDouble("Arm:ShoulderOffset", shoulderOffset);
    prefs.putDouble("Arm:ElbowOffset", elbowOffset);

    zeroElbowMotorEncoder();
    zeroShoulderMotorEncoder();
  }

  /**
   * Updates all arm related values on the dashboard
   */
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
    SmartDashboard.putNumber("Arm:distFromBase", getDistanceFromBase());

    SmartDashboard.putNumber("Arm:shoulder offset", shoulderOffset);
    SmartDashboard.putNumber("Arm:elbow offset", elbowOffset);
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
    return (int) (degrees * 4096 / 360 + shoulderOffset);
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
  public int getShoulderAbsoluteCounts() {
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
  public int getElbowAbsoluteCounts() {
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
  public double getShoulderAbsoluteDegrees() {
    double auxEncoderPos = convertShoulderAbsCountsToDegrees(getShoulderAbsoluteCounts());
    double normalizedPos = (auxEncoderPos + 180) > 0 ? 
        (auxEncoderPos + 180) % 360. - 180 : 
        (auxEncoderPos + 180) % 360. + 180;
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
  public double getElbowAbsoluteDegrees() {
    double auxEncoderPos = convertElbowAbsCountsToDegrees(getElbowAbsoluteCounts());
    double normalizedPos = (auxEncoderPos + 180) > 0 ? 
        (auxEncoderPos + 180) % 360. - 180 : 
        (auxEncoderPos + 180) % 360. + 180;
    return normalizedPos;
  }

  /**
   * Get the elbow degrees as read by the motor encoder (before springs)
   */
  public double getElbowMotorDegrees() {
    return convertElbowMotorCountsToDegrees(getElbowMotorCounts());
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

  public boolean targetReached() {
    return Math.abs(convertMotorShoulderToDegrees(shoulder.getClosedLoopError())) < RobotMap.ARM_TOLERANCE
        && Math.abs(convertMotorElbowToDegrees(elbow.getClosedLoopError())) < RobotMap.ARM_TOLERANCE;
  }

  public void setShoulderSpeed(int speed) {
    shoulder.configMotionCruiseVelocity(speed * 4096 * 4 / 360 / 10);
    shoulder.configMotionAcceleration((int)(speed * 4096 * 4 / 360 / 10 * 1.5));
  }

  public void setElbowSpeed(int speed) {
    elbow.configMotionCruiseVelocity(speed * 4096 * 4 / 360 / 10);
    elbow.configMotionAcceleration((int)(speed * 4096 * 4 / 360 / 10 * 1.5));
  }

  public void setShoulder(double percentOutput) {
    shoulder.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setElbow(double percentOutput) {
    elbow.set(ControlMode.PercentOutput, percentOutput);
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
    double normalizedPos = (auxEncoderPos + 180) > 0 ? 
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
    return isSafePosition(targetShoulderAngle, targetElbowAngle, false);
  }

  public boolean isSafePosition(double [] target) {
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

  public double getDistanceFromBase() {
    double supportX = 0;
    double supportY = ARM_HEIGHT;
    double shoulderX = SHOULDER_LENGTH * Math.sin(Math.toRadians(getMotorShoulderDegrees()));
    double shoulderY = SHOULDER_LENGTH * Math.cos(Math.toRadians(getMotorShoulderDegrees()));
    double elbowX = ELBOW_LENGTH * Math.sin(Math.toRadians(getMotorElbowDegrees()));
    double elbowY = ELBOW_LENGTH * Math.cos(Math.toRadians(getMotorElbowDegrees()));

    double x1 = supportX + shoulderX + elbowX;
    double y1 = supportY + shoulderY + elbowY;

    double x2 = supportX + shoulderX;
    double y2 = supportY + shoulderY;

    return Math.max(Math.sqrt(x1*x1 + y1*y1), Math.sqrt(x2*x2 + y2*y2));

  }

public boolean shoulderSkipped() {
  double auxEncoderPos = convertShoulderToDegrees(getShoulderAbsolutePosition());
  double normalizedPos = (auxEncoderPos + 180) > 0 ? 
      (auxEncoderPos + 180) % 360. - 180 : 
      (auxEncoderPos + 180) % 360. + 180;
	return Math.abs(normalizedPos - getMotorShoulderDegrees()) > 45;
}

public boolean elbowSkipped() {
	double auxEncoderPos = convertElbowToDegrees(getElbowAbsolutePosition());
  double normalizedPos = (auxEncoderPos + 180) > 0 ? 
      (auxEncoderPos + 180) % 360. - 180 : 
      (auxEncoderPos + 180) % 360. + 180;
	return Math.abs(normalizedPos - getMotorElbowDegrees()) > 45;
}

}

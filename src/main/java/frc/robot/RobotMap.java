/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  public static final int ELBOW_ID = 0;
  public static final int SHOULDER_ID = 1;
  public static final int LEFT_MASTER_DRIVE_ID = 0;
  public static final int RIGHT_MASTER_DRIVE_ID = 11;
  public static final int LEFT_FOLLOWER00_DRIVE_ID = 1;
  public static final int LEFT_FOLLOWER01_DRIVE_ID = 2;
  public static final int LEFT_FOLLOWER02_DRIVE_ID = 3;
  public static final int RIGHT_FOLLOWER00_DRIVE_ID = 13;
  public static final int RIGHT_FOLLOWER01_DRIVE_ID = 14;
  public static final int RIGHT_FOLLOWER02_DRIVE_ID = 15;
  public static final int SAPG_MOTOR_ID = 10;
  public static final int SHOULDER_CANIFIER_ID = 11;
  public static final int ELBOW_CANIFIER_ID = 12;
  public static final int INTAKE_ID = 13;
  public static final int CLIMBER_ID = 2;
  public static final int SAPG_DEPLOY_FORWARD = 2;
  public static final int SAPG_DEPLOY_REVERSE = 3;
  public static final int SAPG_GRAB_FORWARD = 4;
  public static final int SAPG_GRAB_REVERSE = 5;

  public static final double ARM_TOLERANCE = 1;
  public static final double DRIVE_VOLTAGE_LIMIT = 0.83;

  public static final int CAN_TIMEOUT = 1;

  public static final int MAX_SPEED = 14500;
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;
public static final int NUM_DRIVE_MOTORS_PER_SIDE = 0;
public static final double LOW_DRIVE_RATIO = 0;
public static final double HIGH_DRIVE_RATIO = 0;
public static final double DRIVE_SENSOR_RATIO = 0;
public static final double DRIVE_LOW_STATIC_FRICTION_VOLTAGE = 0;
public static final double DRIVE_HIGH_STATIC_FRICTION_VOLTAGE = 0;
public static final double DRIVE_LOW_EFFICIENCY = 0;
public static final double DRIVE_HIGH_EFFICIENCY = 0;
public static final double DRIVE_VEL_LOW_KI = 0;
public static final double DRIVE_VEL_LOW_KP = 0;
public static final double DRIVE_VEL_LOW_KD = 0;
public static final double DRIVE_VEL_LOW_IZONE = 0;
public static final double DRIVE_VEL_LOW_IMAX = 0;
public static final int SHIFTER_LEFT_OUT = 7;
public static final int SHIFTER_LEFT_IN = 0;
public static final int SHIFTER_RIGHT_OUT = 6;
public static final int SHIFTER_RIGHT_IN = 1;
public static final int SHIFTER_LEFT_PCM = 0;
public static final int SHIFTER_RIGHT_PCM = 0;
public static final double MAX_DRIVE_VOLTAGE = 0;
public static final double DRIVE_CURRENT_LIMIT = 0;
public static final double MAX_ACCEL_LOW = 0;
public static final double SHIFT_INTO_LOW_GEAR = 0;
public static final double SHIFT_INTO_HIGH_GEAR = 0;
public static final double COMMANDED_STOP_SPEED = 0;
public static final double SHIFT_INTO_LOW_GEAR_STOP = 0;
public static final double DRIVE_VEL_HIGH_KP = 0;
public static final double DRIVE_VEL_HIGH_KI = 0;
public static final double DRIVE_VEL_HIGH_KD = 0;
public static final double DRIVE_VEL_HIGH_IZONE = 0;
public static final double DRIVE_VEL_HIGH_IMAX = 0;
public static final double MAX_ACCEL_HIGH = 0;
public static final int OPERATOR_CONTROLLER_PORT = 1;
public static final int DRIVE_CONTROLLER_PORT = 0;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}

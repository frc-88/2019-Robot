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

  /////////////////////////////////////////////////////////////////////////////
  // IDs
  /////////////////////////////////////////////////////////////////////////////


  // Drivetrain
  public static final int LEFT_MASTER_DRIVE_ID = 0;
  public static final int RIGHT_MASTER_DRIVE_ID = 11;
  public static final int LEFT_FOLLOWER00_DRIVE_ID = 1;
  public static final int LEFT_FOLLOWER01_DRIVE_ID = 2;
  public static final int LEFT_FOLLOWER02_DRIVE_ID = 3;
  public static final int RIGHT_FOLLOWER00_DRIVE_ID = 13;
  public static final int RIGHT_FOLLOWER01_DRIVE_ID = 14;
  public static final int RIGHT_FOLLOWER02_DRIVE_ID = 15;

  public static final int SHIFTER_LEFT_OUT = 0;
  public static final int SHIFTER_LEFT_IN = 7;
  public static final int SHIFTER_RIGHT_OUT = 1;
  public static final int SHIFTER_RIGHT_IN = 6;
  public static final int SHIFTER_LEFT_PCM = 21;
  public static final int SHIFTER_RIGHT_PCM = 21;

  // Arm
  public static final int ELBOW_ID = 04;
  public static final int SHOULDER_ID = 05;
  public static final int SHOULDER_CANIFIER_ID = 20;
  public static final int ELBOW_CANIFIER_ID = 21;

  // Intake
  
  public static final int INTAKE_ID = 10;

  // Climber
  public static final int CLIMBER_ID = 12;

  // SAPG
  public static final int SAPG_DEPLOY_FORWARD = 2;
  public static final int SAPG_DEPLOY_REVERSE = 5;
  public static final int SAPG_GRAB_FORWARD = 4;
  public static final int SAPG_GRAB_REVERSE = 3;
  public static final int SAPG_DEPLOY_PCM = 21;
  public static final int SAPG_GRAB_PCM = 21;
  public static final int SAPG_MOTOR_ID = 06;

  // Compressor
  public static final int COMPRESSOR_PCM = 22;

  /////////////////////////////////////////////////////////////////////////////
  // Other constants
  /////////////////////////////////////////////////////////////////////////////

  // Drive
  public static final int NUM_DRIVE_MOTORS_PER_SIDE = 4;
  public static final double LOW_DRIVE_RATIO = (1 / 60.16) * 6 * 3.14159;
  public static final double HIGH_DRIVE_RATIO = (1 / 17.82) * 6 * 3.14159;
  public static final double DRIVE_SENSOR_RATIO = (1 / (6 * 3.14159)) * (80/12) * 3;

  public static final double DRIVE_LOW_STATIC_FRICTION_VOLTAGE = .53;
  public static final double DRIVE_HIGH_STATIC_FRICTION_VOLTAGE = .75;
  public static final double DRIVE_LOW_EFFICIENCY = 0;
  public static final double DRIVE_HIGH_EFFICIENCY = 0;

  public static final double DRIVE_VEL_LOW_KI = 0;
  public static final double DRIVE_VEL_LOW_KP = 0;
  public static final double DRIVE_VEL_LOW_KD = 0;
  public static final double DRIVE_VEL_LOW_IZONE = 0;
  public static final double DRIVE_VEL_LOW_IMAX = 0;

  public static final double MAX_DRIVE_VOLTAGE = 0;
  public static final double MAX_SPEED_LOW = 8;
  public static final double MAX_SPEED_HIGH = 15;
  public static final double DRIVE_CURRENT_LIMIT = 0;
  public static final double MAX_ACCEL_LOW = 0;
  public static final double MAX_ACCEL_HIGH = 0;

  public static final double SHIFT_INTO_LOW_GEAR = 0;
  public static final double SHIFT_INTO_HIGH_GEAR = 0;
  public static final double COMMANDED_STOP_SPEED = 0;
  public static final double SHIFT_INTO_LOW_GEAR_STOP = 0;

  public static final double DRIVE_VEL_HIGH_KP = 0;
  public static final double DRIVE_VEL_HIGH_KI = 0;
  public static final double DRIVE_VEL_HIGH_KD = 0;
  public static final double DRIVE_VEL_HIGH_IZONE = 0;
  public static final double DRIVE_VEL_HIGH_IMAX = 0;

  public static final int DRIVE_MIN_TRAJ_POINTS = 10;


  // Arm
  public static final double ARM_TOLERANCE = 1;


  // Misc
  public static final int CAN_TIMEOUT = 0;

  public static final int OPERATOR_CONTROLLER_PORT = 1;
  public static final int DRIVE_CONTROLLER_PORT = 0;

}

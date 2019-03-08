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
  public static final int LEFT_TALON_FOLLOWER_DRIVE_ID = 3;
  public static final int LEFT_VICTOR_FOLLOWER00_DRIVE_ID = 1;
  public static final int LEFT_VICTOR_FOLLOWER01_DRIVE_ID = 2;
  public static final int RIGHT_TALON_FOLLOWER_DRIVE_ID = 13;
  public static final int RIGHT_VICTOR_FOLLOWER00_DRIVE_ID = 14;
  public static final int RIGHT_VICTOR_FOLLOWER01_DRIVE_ID = 15;

  public static final int SHIFTER_LEFT_OUT = 0;
  public static final int SHIFTER_LEFT_IN = 7;
  public static final int SHIFTER_RIGHT_OUT = 1;
  public static final int SHIFTER_RIGHT_IN = 6;
  public static final int SHIFTER_LEFT_PCM = 21;
  public static final int SHIFTER_RIGHT_PCM = 21;

  // Arm
  public static final int ELBOW_ID = 04;
  public static final int SHOULDER_ID = 05;
  public static final int SHOULDER_AUXILARY_ID = 3;
  public static final int ELBOW_AUXILARY_ID = 13;

  // Intake
  
  public static final int INTAKE_ID = 10;
  public static final int INTAKE_IR_ID = 0;

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
  public static final int SAPG_PANEL_IR_ID = 2;

  // Compressor
  public static final int COMPRESSOR_PCM = 22;

  // Pidgeon
  public static final int PIDGEON_ID = 24;

  /////////////////////////////////////////////////////////////////////////////
  // Other constants
  /////////////////////////////////////////////////////////////////////////////

  // Drive
  public static final int NUM_DRIVE_MOTORS_PER_SIDE = 4;
  public static final double LOW_DRIVE_RATIO = (1. / 60.16) * (6.25/12.) * 3.14159;
  public static final double HIGH_DRIVE_RATIO = (1. / 27.82) * (6.25/12.) * 3.14159;
  public static final double DRIVE_SENSOR_RATIO = (1. / ((6.25/12.) * 3.14159)) * (54.0/30.0) * 3.;

  public static final double DRIVE_LOW_STATIC_FRICTION_VOLTAGE = .53;
  public static final double DRIVE_HIGH_STATIC_FRICTION_VOLTAGE = .75;
  public static final double DRIVE_LEFT_LOW_EFFICIENCY = .95;
  public static final double DRIVE_LEFT_HIGH_EFFICIENCY = .95;
  public static final double DRIVE_RIGHT_LOW_EFFICIENCY =.9;
  public static final double DRIVE_RIGHT_HIGH_EFFICIENCY = .9;

  public static final double DRIVE_VEL_LOW_KI = 0.016;
  public static final double DRIVE_VEL_LOW_KP = 0;
  public static final double DRIVE_VEL_LOW_KD = 0;
  public static final double DRIVE_VEL_LOW_IZONE = 3;
  public static final double DRIVE_VEL_LOW_IMAX = 3;

  public static final double MAX_DRIVE_VOLTAGE = 11;
  public static final double MAX_SPEED_LOW = 6.25;
  public static final double MAX_SPEED_HIGH = 13.5;
  public static final double MAX_SPEED_FORCE_LOW = 4;

  public static final double DRIVE_CURRENT_LIMIT = 200;
  public static final double MAX_ACCEL_LOW = 2.2;
  public static final double MAX_ACCEL_HIGH = 4.5;
  public static final double MAX_ACCEL_LOW_TIPPY = 1.8;
  public static final double MAX_ACCEL_HIGH_TIPPY = 3.5;
  public static final double ARM_TIPPY_DISTANCE = 66;

  public static final double SHIFT_INTO_LOW_GEAR = 4;
  public static final double SHIFT_INTO_HIGH_GEAR = 5.4;
  public static final double COMMANDED_STOP_SPEED = 1;
  public static final double SHIFT_INTO_LOW_GEAR_STOP = 5;

  public static final double DRIVE_VEL_HIGH_KP = 0;
  public static final double DRIVE_VEL_HIGH_KI = 0.01;
  public static final double DRIVE_VEL_HIGH_KD = 0;
  public static final double DRIVE_VEL_HIGH_IZONE = 3;
  public static final double DRIVE_VEL_HIGH_IMAX = 3;

  public static final int DRIVE_SPEED_EXP = 3;
  public static final int DRIVE_TURN_EXP = 2;

  public static final int DRIVE_MIN_TRAJ_POINTS = 10;

  // Arm
  public static final double ARM_TOLERANCE = 1;
  public static final int SHOULDER_MAX_SPEED = 120;
  public static final int ELBOW_MAX_SPEED = 160;

  // Climber

  public static final double CLIMBER_LIFTING_CURRENT = 10.0;
  public static final double CLIMBER_SEEKING_SPEED = -1;
  public static final double CLIMBER_SEEKING_RAMP = -0.05;
  public static final int CLIMBER_TICKS_PER_INCH = 15908;
  public static final int CLIMBER_MAX_SPEED = 10;
  public static final double CLIMBER_TOLERANCE = 0.25;

  //Intake
  public static final double INTAKE_HAS_CARGO = 18;

  // OI  
  public static final int OPERATOR_CONTROLLER_PORT = 1;
  public static final int OPERATOR_NONE = 0;
  public static final int OPERATOR_SAPG_TEST = 1;
  public static final int OPERATOR_ARM_TEST = 2;
  public static final int OPERATOR_CONTROL = OPERATOR_NONE;

  public static final int DRIVE_CONTROLLER_PORT = 0;

  public static final int BUTTON_BOX_PORT = 3;

  // Misc
  public static final int CAN_TIMEOUT = 0;

}

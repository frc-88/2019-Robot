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
  public static final int LEFT_MASTER_DRIVE_ID = 2;
  public static final int RIGHT_MASTER_DRIVE_ID = 3;
  public static final int LEFT_FOLLOWER00_DRIVE_ID = 4;
  public static final int LEFT_FOLLOWER01_DRIVE_ID = 5;
  public static final int LEFT_FOLLOWER02_DRIVE_ID = 6;
  public static final int RIGHT_FOLLOWER00_DRIVE_ID = 7;
  public static final int RIGHT_FOLLOWER01_DRIVE_ID = 8;
  public static final int RIGHT_FOLLOWER02_DRIVE_ID = 9;
  public static final int SAPG_MOTOR_ID = 10;
  public static final int SHOULDER_CANIFIER_ID = 11;
  public static final int ELBOW_CANIFIER_ID = 12;
  public static final int INTAKE_ID = 13;
  public static final int SAPG_DEPLOY_FORWARD = 0;
  public static final int SAPG_DEPLOY_REVERSE = 1;
  public static final int SAPG_GRAB_FORWARD = 2;
  public static final int SAPG_GRAB_REVERSE = 3;

  public static final double ARM_TOLERANCE = 1;
  public static final double DRIVE_VOLTAGE_LIMIT = 0.83;

  public static final int CAN_TIMEOUT = 1;

  public static final int MAX_SPEED = 14500;

  public static final double SAPG_POT_SCALAR = 1 / 1024 * 10 * 1.432/2/12;
  public static final double SAPG_POT_OFFSET = 0;
  public static final double SAPG_SIDE_RANGE = 8.5/12;
}

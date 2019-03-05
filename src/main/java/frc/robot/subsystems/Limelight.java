/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.util.LLCameraTransform;
import jaci.pathfinder.Waypoint;

/**
 * 
 * When the green light dims
 * And all the cargo is safe
 * What does Limelight see?
 * 
 */

public class Limelight extends Subsystem {
  private NetworkTable _table;
  private NetworkTableEntry _ta;
  private NetworkTableEntry _tv;
  private NetworkTableEntry _tx;
  private NetworkTableEntry _ty;
  private NetworkTableEntry _ts;
  private NetworkTableEntry _camtran;
  private NetworkTableEntry _ledMode;
  private NetworkTableEntry _pipeline;
  private NetworkTableEntry _getpipe;

  private String name;

  // Variables used for distance calculation by vertical angle offset.
  public double limelight_height = 0;
  public double target_height = 0;
  public double limelight_angle = 0;

  // Variables used for distance calculation by target area.
  public double known_target_area = 1.75;
  public double known_target_distance = 34.5;

  /**
   * Construct a Limelight instance with the default NetworkTables table name.
   */
  public Limelight() {
    this("limelight");
  }

  public Limelight(String network_table_name) {
    name = network_table_name;
    _table = NetworkTableInstance.getDefault().getTable(network_table_name);
    _pipeline = _table.getEntry("pipeline");
    _getpipe = _table.getEntry("getpipe");
    _tv = _table.getEntry("tv");
    _ta = _table.getEntry("ta");
    _tx = _table.getEntry("tx");
    _ty = _table.getEntry("ty");
    _ts = _table.getEntry("ts");
    _camtran = _table.getEntry("camtran");
    _ledMode = _table.getEntry("ledMode");

    setPipeline(0);
  }

  /**
   * @return True if the Limelight has a recognized target.
   */
  public boolean hasTarget() {
    return _tv.getDouble(0.0) == 1.0;
  }

  /**
   * @return True if data is being received from the Limelight.
   */
  public boolean isConnected() {
    return (_ta.exists() && _tv.exists() && _tx.exists() && _ty.exists());
  }

  /**
   * Get the horizontal offset angle of the target from the center of the camera
   * frame. If no target is seen, returns zero.
   * 
   * @return A measurement in degrees in the range [-27, 27]
   */
  public double getHorizontalOffsetAngle() {
    return _tx.getDouble(0.0);
  }

  /**
   * Get the vertical offset angle of the target from the center of the camera
   * frame. If no target is seen, returns zero.
   * 
   * @return A measurement in degrees in the range [-20.5, 20.5]
   */
  public double getVerticalOffsetAngle() {
    return _ty.getDouble(0.0);
  }

  /**
   * Get the area of the target as a percentage of the total camera frame. If no
   * target is seen, returns zero.
   * 
   * @return A percentage in the range of [0, 1]
   */
  public double getTargetArea() {
    return _ta.getDouble(0.0);
  }

  /**
   * Get the skew or rotation
   * 
   * @return -90 degrees to 0 degrees
   */
  public double getRotation() {
    return _ts.getDouble(0.0);
  }

  public void ledPipeline() {
    _ledMode.setNumber(0);
  }

  public void ledOff() {
    _ledMode.setNumber(1);
  }

  public void ledBlink() {
    _ledMode.setNumber(2);
  }

  public void ledOn() {
    _ledMode.setNumber(3);
  }


  // Read all 6 dimensions of your camera’s transform (x,y,z,pitch,yaw,roll)
  // by reading the “camtran�? networktable number array.

  public LLCameraTransform getCameraTransform() {
    Number [] transform = _camtran.getNumberArray(new Number[0]);

    if (transform.length == 0) {
      transform = new Number [] {0, 0, 0, 0, 0, 0};
    }

    return new LLCameraTransform(transform);
  }

  public double getTargetDistance() {
    LLCameraTransform cam = getCameraTransform();

    return Math.sqrt(Math.pow(cam.x, 2) + Math.pow(cam.z, 2));
  }

  public double getTargetAngle() {
    LLCameraTransform cam = getCameraTransform();

    return Math.toDegrees(Math.atan2(cam.z, cam.x));
  }

  public Waypoint[] generateWaypointsFromVision() {
    LLCameraTransform cam = getCameraTransform();
    double yaw = Robot.m_navx.getYaw();
    double yawOffset = yaw - cam.yaw;

    SmartDashboard.putNumber("x", cam.x);
    SmartDashboard.putNumber("y", cam.y);
    SmartDashboard.putNumber("yawOffset", yawOffset);

    // 3 Waypoints
    return new Waypoint[] { new Waypoint(cam.x, cam.y, cam.yaw), new Waypoint(0, 3, 0), new Waypoint(0, 1.5, 0) };
  }

  public void setPipeline(double pipeline) {
    _pipeline.setDouble(pipeline);
  }

  public double getPipeline() {
    return _getpipe.getDouble(0);
  }

  public void updateDashboard() {
    SmartDashboard.putBoolean(name + ":HasTarget", hasTarget());
    SmartDashboard.putBoolean(name + ":IsConnected", isConnected());
    SmartDashboard.putNumber(name + ":Distance", getTargetDistance());
  }

  @Override
  public void initDefaultCommand() {
  }

}

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
 * A subsystem to interact with a Limelight vision camera over NetworkTables.
 */
public class Limelight extends Subsystem{
  private static final double HORIZONTAL_FOV = 54.0;
  // private static final double VERTICAL_FOV = 41.0;
  private static final double VIEW_PLANE_WIDTH = 2 * Math.tan(Math.toRadians(HORIZONTAL_FOV / 2));
  // private static final double VIEW_PLANE_HEIGHT = 2 * Math.tan(VERTICAL_FOV/2);
  private static final double SINGLE_TARGET_C = 30.8;
  private static final double SINGLE_TARGET_EXP = -0.386;
  private static final double FULL_TARGET_C = 42.158;
  private static final double FULL_TARGET_EXP = -0.424;
  
  private NetworkTable _table;
  private NetworkTableEntry _ta;
  private NetworkTableEntry _tv;
  private NetworkTableEntry _tx;
  private NetworkTableEntry _ty;
  private NetworkTableEntry _ts;
  private NetworkTableEntry _camtran;
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

    // Read all 6 dimensions of your camera’s transform (x,y,z,pitch,yaw,roll)
    // by reading the “camtran�? networktable number array.

  public LLCameraTransform getCameraTransform() {
    return new LLCameraTransform(_camtran.getNumberArray(new Number[0]));
  }

public double getTargetDistanceByCameraTransform() {
  LLCameraTransform cam = getCameraTransform();

  return Math.sqrt(Math.pow(cam.x, 2) + Math.pow(cam.z, 2));
}

public double getTargetAngleByCameraTransform() {
  LLCameraTransform cam = getCameraTransform();

  return Math.toDegrees(Math.atan2(cam.z, cam.x));
}


  /**
   * Calculates the distance to the target based on the vertical offset angle.
   * 
   * This method requires a known target height, This calculation can be
   * inaccurate if the Limelight and the target have a small difference in
   * heights, as the vertical angle will not change significantly with target
   * distance.
   * 
   * If the total angle between the floor and the projected line to the target is
   * zero, returns zero. This is not a valid state for this algorithm.
   * 
   * @return The distance to the target.
   */
  public double getTargetDistanceByVerticalAngle() {
    double height_difference = target_height - limelight_height;
    double total_angle = limelight_angle + getVerticalOffsetAngle();
    if (total_angle == 0.0) {
      // Return early, because tan(0) = 0, and that would have us divide by zero.
      return -1;
    } else {
      return height_difference / Math.tan(Math.toRadians(total_angle));
    }
  }

  /**
   * Calculates the distance to the target baesd on the visible target area.
   * 
   * This method can be less accurate than calculating distance by vertical angle,
   * but may be better when the vertical angle offset is small.
   * 
   * @return The distance to the target.
   */
  public double getTargetDistanceByArea() {
    if (hasTarget()) {
      return FULL_TARGET_C * Math.pow(_ta.getDouble(0.0), FULL_TARGET_EXP);
    } else {
      return -1;
    }
  }

  /**
   * Calculates the angle of the target surface based on the difference in
   * distance of the components of the vision target.
   * 
   * We do this by calculating an x,y coordinate relative to the robot's origin
   * for each component of the vision target. We can the use the slope of the line
   * between those two points to estimate the angle of the target surface.
   * 
   * @return The surface angle of the target.
   */

  public double getTargetSurfaceAngle() {
    if (hasTarget()) {
      double td0 = SINGLE_TARGET_C * Math.pow(_table.getEntry("ta0").getDouble(0.0), SINGLE_TARGET_EXP);
      double tx0 = Math.atan2(1, (VIEW_PLANE_WIDTH / 2) * _table.getEntry("tx0").getDouble(0.0));
      double y0 = td0 * Math.sin(tx0);
      double x0 = td0 * Math.cos(tx0);

      double td1 = SINGLE_TARGET_C * Math.pow(_table.getEntry("ta1").getDouble(0.0), SINGLE_TARGET_EXP);
      double tx1 = Math.atan2(1, (VIEW_PLANE_WIDTH / 2) * _table.getEntry("tx1").getDouble(0.0));
      double y1 = td1 * Math.sin(tx1);
      double x1 = td1 * Math.cos(tx1);

      /*
      SmartDashboard.putNumber("ta0", _table.getEntry("ta0").getDouble(0.0));
      SmartDashboard.putNumber("ta1", _table.getEntry("ta1").getDouble(0.0));
      SmartDashboard.putNumber("tx0", tx0);
      SmartDashboard.putNumber("tx1", tx1);
      SmartDashboard.putNumber("td0", td0);
      SmartDashboard.putNumber("td1", td1);
      SmartDashboard.putNumber("x0", x0);
      SmartDashboard.putNumber("x1", x1);
      SmartDashboard.putNumber("y0", y0);
      SmartDashboard.putNumber("y1", y1);
      SmartDashboard.putNumber("xDelta", x0 - x1);
      SmartDashboard.putNumber("yDelta", y0 - y1);
      */

      double surface_angle = Math.atan2(y0 - y1, x0 - x1);
      
      if (surface_angle > Math.PI/2) {
        surface_angle -= Math.PI;
      } else if (surface_angle < -Math.PI/2) {
        surface_angle += Math.PI;
      }

      return Math.toDegrees(surface_angle);

    } else {
      return -1;
    }
  }

  public Waypoint[] generateWaypointsFromVision() {
    double yaw = Robot.m_navx.getYaw();
    double x = getTargetDistanceByArea()
            * Math.sin(Math.toRadians(getHorizontalOffsetAngle()));
    double y = getTargetDistanceByArea()
            * Math.cos(Math.toRadians(getHorizontalOffsetAngle()));
    double theta = getTargetSurfaceAngle();
    if (theta < -180) {
      theta += 360;
    } else if (theta > 180) {
      theta -= 360;
    }

    double xprime = x - 12 * Math.cos(Math.toRadians(90 - theta));
    double yprime = y - 12 * Math.sin(Math.toRadians(90 - theta));

    SmartDashboard.putNumber("theta", theta);

    // rotate coordinate system based on yaw
    theta = yaw - theta;
    double rotation = Math.toRadians(yaw);
    double newX = x*Math.cos(rotation) - y*Math.sin(rotation);
    double newY = x*Math.sin(rotation) + y*Math.cos(rotation);
    double newXprime = xprime*Math.cos(rotation) - yprime*Math.sin(rotation);
    double newYprime = xprime*Math.sin(rotation) + yprime*Math.cos(rotation);

    // if (Math.abs(theta) < 10) {
    //   theta = 0;
    // } else if (Math.abs(theta)-30 < 10) {
    //   theta = 30 * Math.signum(theta);
    // } else if (Math.abs(theta)-90 < 10) {
    //   theta = 90 * Math.signum(theta);
    // } else if (Math.abs(theta)-150 < 10) {
    //   theta = 150 * Math.signum(theta);
    // } else if (Math.abs(theta)-180 < 10) {
    //   theta = 180 * Math.signum(theta);
    // }

      SmartDashboard.putNumber("x", x);
      SmartDashboard.putNumber("xprime", xprime);
      SmartDashboard.putNumber("y", y);
      SmartDashboard.putNumber("yprime", yprime);
      SmartDashboard.putNumber("newx", newX);
      SmartDashboard.putNumber("newxprime", newXprime);
      SmartDashboard.putNumber("newy", newY);
      SmartDashboard.putNumber("newyprime", newYprime);
      SmartDashboard.putNumber("theta_final", theta);

    // 3 Waypoints
    return new Waypoint[] { 
          new Waypoint(0, 0, yaw), 
          new Waypoint(xprime, yprime, theta), 
          new Waypoint(x, y, theta)
    };
  }



  public void setPipeline(double pipeline) {
    _pipeline.setDouble(pipeline);
  }

  public double getPipeline() {
    return _getpipe.getDouble(0);
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("LL:" + name + ":Angle(Transform)", getTargetAngleByCameraTransform());
    SmartDashboard.putNumber("LL:" + name + ":Distance(Transform)", getTargetDistanceByCameraTransform());
  }

  @Override
  public void initDefaultCommand() {
  }

}

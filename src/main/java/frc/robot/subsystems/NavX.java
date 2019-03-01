/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;

/**
 * 
 * North, south, east and west
 * Forward, back, and side to side
 * We know where we are
 * 
 */
public class NavX extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private AHRS m_navx = new AHRS(SPI.Port.kMXP);

  private double pitchOffset = 0;

  public NavX() {

  }

  public void zeroYaw() {
    m_navx.zeroYaw();
  }

  public double getYaw() {
    return m_navx.getYaw();
  }

  public void zeroPitch() {
    pitchOffset = -m_navx.getRoll();
  }

  public double getPitch() {
    return m_navx.getRoll() + pitchOffset;
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("NavX Yaw", getYaw());
    SmartDashboard.putNumber("NavX Pitch", getPitch());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

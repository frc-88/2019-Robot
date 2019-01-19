/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import javax.print.attribute.standard.RequestingUserName;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.robot.*;
import frc.robot.commands.*;
import frc.robot.Utils.*;

/**
 * Add your docs here.
 */
public class Drive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands

  TalonSRX left, right;
  VictorSPX leftFollower0, leftFollower1, leftFollower2, rightFollower0, rightFollower1, rightFollower2;

  NetworkTableEntry leftCommandedVoltage, rightCommandedVoltage, leftSpd, rightSpd, leftCurrent, rightCurrent;

  public Drive() {
    left = new TalonSRX(RobotMap.LEFT_MASTER_DRIVE_ID);
    right = new TalonSRX(RobotMap.RIGHT_MASTER_DRIVE_ID);
    leftFollower0 = new VictorSPX(RobotMap.LEFT_FOLLOWER00_DRIVE_ID);
    leftFollower1 = new VictorSPX(RobotMap.LEFT_FOLLOWER01_DRIVE_ID);
    leftFollower2 = new VictorSPX(RobotMap.LEFT_FOLLOWER02_DRIVE_ID);
    rightFollower0 = new VictorSPX(RobotMap.RIGHT_FOLLOWER00_DRIVE_ID);
    rightFollower1 = new VictorSPX(RobotMap.RIGHT_FOLLOWER01_DRIVE_ID);
    rightFollower2 = new VictorSPX(RobotMap.RIGHT_FOLLOWER02_DRIVE_ID);

    leftFollower0.follow(left);
    leftFollower1.follow(left);
    leftFollower2.follow(left);
    rightFollower0.follow(right);
    rightFollower1.follow(right);
    rightFollower2.follow(right);

    left.configPeakOutputForward(RobotMap.DRIVE_VOLTAGE_LIMIT, RobotMap.CAN_TIMEOUT);
    left.configPeakOutputReverse(-RobotMap.DRIVE_VOLTAGE_LIMIT, RobotMap.CAN_TIMEOUT);
    left.configContinuousCurrentLimit(25, RobotMap.CAN_TIMEOUT);
    left.configPeakCurrentLimit(25, RobotMap.CAN_TIMEOUT);
    left.configPeakCurrentDuration(0, RobotMap.CAN_TIMEOUT);
    left.enableCurrentLimit(false);
    left.configVoltageCompSaturation(12, RobotMap.CAN_TIMEOUT);
    left.enableVoltageCompensation(false);
    // left.configPeakOutputForward(1, RobotMap.CAN_TIMEOUT);
    // left.configPeakOutputReverse(-1, RobotMap.CAN_TIMEOUT);
    // left.enableCurrentLimit(false);

    left.configOpenloopRamp(.125, RobotMap.CAN_TIMEOUT);
    left.configClosedloopRamp(0, RobotMap.CAN_TIMEOUT);
    // left.configOpenloopRamp(0, RobotMap.CAN_TIMEOUT);
    // left.configClosedloopRamp(0, RobotMap.CAN_TIMEOUT);

    right.configPeakOutputForward(RobotMap.DRIVE_VOLTAGE_LIMIT, RobotMap.CAN_TIMEOUT);
    right.configPeakOutputReverse(-RobotMap.DRIVE_VOLTAGE_LIMIT, RobotMap.CAN_TIMEOUT);
    right.configContinuousCurrentLimit(25, RobotMap.CAN_TIMEOUT);
    right.configPeakCurrentLimit(25, RobotMap.CAN_TIMEOUT);
    right.configPeakCurrentDuration(0, RobotMap.CAN_TIMEOUT);
    right.enableCurrentLimit(false);
    right.configVoltageCompSaturation(12, RobotMap.CAN_TIMEOUT);
    right.enableVoltageCompensation(false);
    // right.configPeakOutputForward(1, RobotMap.CAN_TIMEOUT);
    // right.configPeakOutputReverse(-1, RobotMap.CAN_TIMEOUT);
    // right.enableCurrentLimit(false);

    right.configOpenloopRamp(.125, RobotMap.CAN_TIMEOUT);
    right.configClosedloopRamp(0, RobotMap.CAN_TIMEOUT);
    // right.configOpenloopRamp(0, RobotMap.CAN_TIMEOUT);
    // right.configClosedloopRamp(0, RobotMap.CAN_TIMEOUT);

    left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, RobotMap.CAN_TIMEOUT);
    right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, RobotMap.CAN_TIMEOUT);

    left.configMotionAcceleration(3000, RobotMap.CAN_TIMEOUT);
    right.configMotionAcceleration(3000, RobotMap.CAN_TIMEOUT);
    right.configMotionCruiseVelocity(2500, RobotMap.CAN_TIMEOUT);
    left.configMotionCruiseVelocity(2500, RobotMap.CAN_TIMEOUT);


    leftCommandedVoltage = Robot.dataTable.getEntry("leftCommandVoltage");
    rightCommandedVoltage = Robot.dataTable.getEntry("rightCommandVoltage");
    leftSpd = Robot.dataTable.getEntry("leftSpd");
    rightSpd = Robot.dataTable.getEntry("rightSpd");
    leftCurrent = Robot.dataTable.getEntry("leftCurrent");
    rightCurrent = Robot.dataTable.getEntry("rightCurrent");
  }

  public void drive(double leftSpd, double rightSpd) {
    left.set(ControlMode.PercentOutput, -leftSpd);
    right.set(ControlMode.PercentOutput, rightSpd);
  }


  public void resetEncoders() {
    left.setSelectedSensorPosition(0, 0, RobotMap.CAN_TIMEOUT);
    right.setSelectedSensorPosition(0, 0, RobotMap.CAN_TIMEOUT);
  }

  public void tankDrive(double leftSpd, double rightSpd) {
    // System.out.println(leftSpd + " " + rightSpd);
    // leftSpd = DriveUtils.signedPow(leftSpd,RobotMap.DRIVETRAIN_SCALING_EXPONENT);
    // rightSpd =
    // DriveUtils.signedPow(rightSpd,RobotMap.DRIVETRAIN_SCALING_EXPONENT);
    // drive(leftSpd, rightSpd);
    double turn = (rightSpd - leftSpd) / 2;
    double spd = (rightSpd + leftSpd) / 2;

    // System.out.println(turn + " " + spd);

    leftSpd = spd - turn;
    rightSpd = spd + turn;

    // leftSpd *= RobotMap.MAX_SPEED;
    // rightSpd *= RobotMap.MAX_SPEED;

    drive(leftSpd, rightSpd);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TankDrive());
  }

  public void sendData() {
    leftCommandedVoltage.setDouble(left.getMotorOutputVoltage());
    rightCommandedVoltage.setDouble(right.getMotorOutputVoltage());
    leftSpd.setDouble(left.getSelectedSensorVelocity(0) * 2 / 2367.488);
    rightSpd.setDouble(right.getSelectedSensorVelocity(0) * 2 / 2367.488);
    leftCurrent.setDouble(left.getOutputCurrent());
    rightCurrent.setDouble(right.getOutputCurrent());
  }

  public double leftEncoderValue() {
    return -left.getSelectedSensorPosition(0);
  }

  public double rightEncoderValue() {
    return right.getSelectedSensorPosition(0);
  }
}
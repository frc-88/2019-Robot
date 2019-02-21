package frc.robot.driveutil;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;

import frc.robot.RobotMap;

public class DriveConfiguration {
    public TJDriveModuleConfiguration left, right;

    private TalonSRXConfiguration _talonMaster;
    private TalonSRXConfiguration _talonFollower;
    private VictorSPXConfiguration _victorFollower;

    public DriveConfiguration()
    {
        left = new TJDriveModuleConfiguration();
        right = new TJDriveModuleConfiguration();

        left.master = RobotMap.LEFT_MASTER_DRIVE_ID;
        left.talonFollowers = new int[] {RobotMap.LEFT_TALON_FOLLOWER_DRIVE_ID};
        left.victorFollowers = new int [] {RobotMap.LEFT_VICTOR_FOLLOWER00_DRIVE_ID,
            RobotMap.LEFT_VICTOR_FOLLOWER01_DRIVE_ID};

        right.master = RobotMap.RIGHT_MASTER_DRIVE_ID;
        right.talonFollowers = new int[] {RobotMap.RIGHT_TALON_FOLLOWER_DRIVE_ID};
        right.victorFollowers = new int [] {RobotMap.RIGHT_VICTOR_FOLLOWER00_DRIVE_ID,
            RobotMap.RIGHT_VICTOR_FOLLOWER01_DRIVE_ID};

        _talonMaster = new TalonSRXConfiguration();
        _talonFollower = new TalonSRXConfiguration();
        _victorFollower = new VictorSPXConfiguration();

        /* Talon SRX - Master */
        _talonMaster.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder; 
        _talonMaster.neutralDeadband = 0.01;
        _talonMaster.voltageCompSaturation = 12;

        left.masterConfiguration = _talonMaster;
        right.masterConfiguration = _talonMaster;

        /* Followers */

        _talonFollower.neutralDeadband = 0.01;
        _victorFollower.neutralDeadband = 0.01;

        _talonFollower.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Absolute;

        left.talonFollowerConfiguration = _talonFollower;
        right.talonFollowerConfiguration = _talonFollower;
        left.victorFollowerConfiguration = _victorFollower;
        right.victorFollowerConfiguration = _victorFollower;

        /* General Settings */
        left.neutralMode = NeutralMode.Brake;
        left.invertMotor = true;
        left.enableVoltageCompensation = true;
        right.neutralMode = NeutralMode.Brake;
        right.invertMotor = false;
        right.enableVoltageCompensation = true;
    }
}
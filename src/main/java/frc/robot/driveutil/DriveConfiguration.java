package frc.robot.driveutil;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;

public class DriveConfiguration {
    public TJDriveModuleConfiguration left, right;

    private TalonSRXConfiguration _talonMaster;
    private TalonSRXConfiguration _talonFollower;
    private VictorSPXConfiguration _victorFollower;

    public DriveConfiguration()
    {
        left = new TJDriveModuleConfiguration();
        right = new TJDriveModuleConfiguration();

        left.master = 10;
        left.talonFollowers = new int [] {11, 12, 13};

        right.master = 14;
        right.talonFollowers = new int [] {15, 16, 17};

        _talonMaster = new TalonSRXConfiguration();
        _talonFollower = new TalonSRXConfiguration();
        _victorFollower = new VictorSPXConfiguration();

        /* Talon SRX - Master */
        _talonMaster.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder; 
        _talonMaster.neutralDeadband = 0.01;
        _talonMaster.voltageCompSaturation = 12;

        left.masterConfiguration = _talonMaster;
        right.masterConfiguration = _talonMaster;

        /* Talon SRX - Follower */

        _talonFollower.neutralDeadband = 0.01;

        left.talonFollowerConfiguration = _talonFollower;
        right.talonFollowerConfiguration = _talonFollower;

        /* General Settings */
        left.neutralMode = NeutralMode.Brake;
        left.invertMotor = false;
        left.enableVoltageCompensation = true;
        right.neutralMode = NeutralMode.Brake;
        right.invertMotor = true;
        right.enableVoltageCompensation = true;
    }
}
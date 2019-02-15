/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.driveutil;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;

/**
 * Add your docs here.
 */
public class TJDriveModuleConfiguration {
    public int master;
    public int talonFollowers[]= new int[0];
    public int victorFollowers[]= new int[0];

    public boolean enableCurrentLimit = false;
    public boolean enableVoltageCompensation = false;
    public boolean invertMotor = false;
    public boolean invertSensor = false;
    public NeutralMode neutralMode = NeutralMode.Coast;
    
    public TalonSRXConfiguration masterConfiguration;
    public TalonSRXConfiguration talonFollowerConfiguration;
    public VictorSPXConfiguration victorFollowerConfiguration;
}
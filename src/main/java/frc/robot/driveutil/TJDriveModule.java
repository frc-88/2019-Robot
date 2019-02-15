/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.driveutil;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.util.transmission.Transmission;

/**
 * Add your docs here.
 */
public class TJDriveModule extends TalonSRX {
    private TalonSRX talonFollowers[];
    private VictorSPX victorFollowers[];

    private Transmission m_transmission;

    public TJDriveModule(TJDriveModuleConfiguration config, Transmission transmission) {
        super(config.master);
        this.configFactoryDefault();
        this.configAllSettings(config.masterConfiguration);
        this.enableCurrentLimit(config.enableCurrentLimit);
        this.enableVoltageCompensation(config.enableVoltageCompensation);
        this.setInverted(config.invertMotor);
        this.setSensorPhase(config.invertSensor);
        this.setNeutralMode(config.neutralMode);

        talonFollowers = new TalonSRX [config.talonFollowers.length];
        for (int i = 0; i < config.talonFollowers.length; i++) {
            talonFollowers[i] = new TalonSRX(config.talonFollowers[i]);
            talonFollowers[i].configFactoryDefault();
            talonFollowers[i].configAllSettings(config.talonFollowerConfiguration);
            talonFollowers[i].follow(this);
            talonFollowers[i].setInverted(config.invertMotor);
            talonFollowers[i].setSensorPhase(config.invertSensor);
            talonFollowers[i].setNeutralMode(config.neutralMode);
        }

        victorFollowers = new VictorSPX [config.victorFollowers.length];
        for (int i = 0; i < config.victorFollowers.length; i++) {
            victorFollowers[i] = new VictorSPX(config.victorFollowers[i]);
            victorFollowers[i].configFactoryDefault();
            victorFollowers[i].configAllSettings(config.victorFollowerConfiguration);
            victorFollowers[i].follow(this);
            victorFollowers[i].setInverted(config.invertMotor);
            victorFollowers[i].setSensorPhase(config.invertSensor);
            victorFollowers[i].setNeutralMode(config.neutralMode);
        }

        m_transmission = transmission;
    }

    /**
     * Get the number of talon followers this drive module has.
     */
    public int getNumTalonFollowers() {
        return talonFollowers.length;
    }

    /**
     * Get the output current of the ith talon follower
     */
    public double getFollowerCurrent(int i) {
        return talonFollowers[i].getOutputCurrent();
    }

    /**
     * Get the total output current of the master and all talon followers
     */
    public double getTotalCurrent() {
        double total = this.getOutputCurrent();
        for (TalonSRX follower : talonFollowers) {
            total += follower.getOutputCurrent();
        }
        return total;
    }

    /**
     * Get the position being read by the sensor on the master, scaled to the
     * output units of the transmission.
     */
    public double getScaledSensorPosition() {
        return m_transmission.convertSensorPositionToOutput(
                this.getSelectedSensorPosition());
    }

    /**
     * Get the velocity being read by the sensor on the master, scaled to the
     * output units of the transmission.
     */
    public double getScaledSensorVelocity() {
        return m_transmission.convertSensorVelocityToOutput(
                this.getSelectedSensorVelocity());
    }


    /**
     * Drives this module to the given output velocity while using a system
     * model of the drivetrain to not command a voltage that would result in
     * exceeding the given current limit.
     * 
     * Parameters:
     * @param targetVelocity The desired velocity at the output of the transmission
     * @param currentLimit The maximum current, in amps, that we want the sum
     *                     of all the motors to draw
     */
    public void setVelocityCurrentLimited(double targetVelocity, double currentLimit) {
        this.set(ControlMode.PercentOutput, 
                m_transmission.getCurrentLimitedVoltage(
                        targetVelocity, this.getSelectedSensorVelocity(), currentLimit)
                    / 12);
    }

}
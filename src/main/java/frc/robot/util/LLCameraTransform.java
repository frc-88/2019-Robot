/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

/**
 * Add your docs here.
 */
public class LLCameraTransform {
    public double x,y,z,pitch,yaw,roll;

    public LLCameraTransform(Number[] rawNumbers) {
        x = rawNumbers[0].doubleValue();
        y = rawNumbers[1].doubleValue();
        z = rawNumbers[2].doubleValue();
        pitch = rawNumbers[3].doubleValue();
        yaw = rawNumbers[4].doubleValue();
        roll = rawNumbers[5].doubleValue();
    }
}

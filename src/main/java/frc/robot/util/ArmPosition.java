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
public class ArmPosition {
        // doubleake position (164, 85)
    // secure position (160, 10)
    // low rocket  (150,0)
    // medium rocket  (87,0)
    // high rocket  (28,0)
    // cargo ship   (105,35)
    // starting config  (164,0)
    //private static final double []  = {, };
    private static final double [] HOME = {160, 10};
    private static final double [] START = {165, 0};
    private static final double [] INTAKE = {160, 85};
    private static final double [] CARGO_SHIP_FRONT = {105, 35};
    private static final double [] CARGO_SHIP_BACK = {-55, -70};
    private static final double [] LOW_ROCKET = {150, 0};
    private static final double [] MEDIUM_ROCKET_FRONT = {85, 0};
    private static final double [] MEDIUM_ROCKET_FRONT2 = {55, 180};
    private static final double [] MEDIUM_ROCKET_BACK = {-55, -180};
    private static final double [] HIGH_ROCKET_FRONT = {28, 0};
    private static final double [] HIGH_ROCKET_BACK = {-28, 0};
    private static final double [] PRE_CLIMB = {80, 180};
}

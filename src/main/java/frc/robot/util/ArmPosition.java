/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import java.util.Arrays;
import java.util.List;

/**
 * Add your docs here.
 */
public final class ArmPosition {
    public static final ArmSetpoint HOME = new ArmSetpoint(158, 2);
    public static final ArmSetpoint START = new ArmSetpoint(158, 2);
    public static final ArmSetpoint INTAKE = new ArmSetpoint(162, 82);
    public static final ArmSetpoint BATTLE_MODE = new ArmSetpoint(165, -10);
    public static final ArmSetpoint CARGO_SHIP_FRONT = new ArmSetpoint(105, 33);
    public static final ArmSetpoint CARGO_SHIP_BACK = new ArmSetpoint(-105, -33);
    public static final ArmSetpoint CARGO_SHIP_BACK2 = new ArmSetpoint(-60, -215);
    public static final ArmSetpoint LOW_ROCKET = new ArmSetpoint(150, 0);
    public static final ArmSetpoint MEDIUM_ROCKET_FRONT = new ArmSetpoint(85, 0);
    public static final ArmSetpoint MEDIUM_ROCKET_FRONT2 = new ArmSetpoint(55, 180);
    public static final ArmSetpoint MEDIUM_ROCKET_BACK = new ArmSetpoint(-85, 0);
    public static final ArmSetpoint MEDIUM_ROCKET_BACK2 = new ArmSetpoint(-29, -180);
    public static final ArmSetpoint HIGH_ROCKET_FRONT = new ArmSetpoint(28, 0);
    public static final ArmSetpoint HIGH_ROCKET_BACK = new ArmSetpoint(-30, 0);
    public static final ArmSetpoint PRE_CLIMB = new ArmSetpoint(75, 176);
    public static final ArmSetpoint PRE_CLIMB2 = new ArmSetpoint(103, 170);
    public static final ArmSetpoint LOW_ROCKET_BACK = new ArmSetpoint(-90, -180);

    public static ArmSetpoint[] getPath(ArmSetpoint currentSetpoint, ArmSetpoint targetSetpoint) {

        if (targetSetpoint.equals(MEDIUM_ROCKET_BACK2) 
                && (currentSetpoint.equals(INTAKE) || currentSetpoint.equals(HOME))) {

            return new ArmSetpoint[] {
                new ArmSetpoint(158, 13),
                targetSetpoint
            };
            
        }
        else if ((targetSetpoint.equals(INTAKE) || targetSetpoint.equals(HOME)) 
                && currentSetpoint.equals(MEDIUM_ROCKET_BACK2)) {

            return new ArmSetpoint[] {
                new ArmSetpoint(158, 13).passShoulder(),
                targetSetpoint
            };
            
        }
        else if (targetSetpoint.equals(LOW_ROCKET_BACK) 
                && (currentSetpoint.equals(INTAKE) || currentSetpoint.equals(HOME))) {

            return new ArmSetpoint[] {
                new ArmSetpoint(158, 13),
                new ArmSetpoint(-35,-180).passShoulder(),
                targetSetpoint
            };
            
        }
        else if ((targetSetpoint.equals(INTAKE) || targetSetpoint.equals(HOME)) 
                && currentSetpoint.equals(LOW_ROCKET_BACK)) {

            return new ArmSetpoint[] {
                new ArmSetpoint(-35,-180).passShoulder(),
                new ArmSetpoint(158, 13),
                targetSetpoint
            };
            
        }
        else if (targetSetpoint.equals(CARGO_SHIP_BACK2)
                && (currentSetpoint.equals(INTAKE) || currentSetpoint.equals(HOME))) {

            return new ArmSetpoint[] {
                new ArmSetpoint(150, 5),
                new ArmSetpoint(0,-145),
                targetSetpoint
            };
    
        }
        else if ((targetSetpoint.equals(INTAKE) || targetSetpoint.equals(HOME))
                && currentSetpoint.equals(CARGO_SHIP_BACK2)) {

            return new ArmSetpoint[] {
                new ArmSetpoint(0,-145),
                new ArmSetpoint(150, 5),
                targetSetpoint
            };
    
        }
        else if (targetSetpoint.equals(PRE_CLIMB) 
                && !(currentSetpoint.equals(PRE_CLIMB) || currentSetpoint.equals(PRE_CLIMB2))) {

            ArmSetpoint[] toIntake = getPath(currentSetpoint, INTAKE);
            ArmSetpoint[] ret = Arrays.copyOf(toIntake, toIntake.length + 1);
            ret[ret.length - 1] = targetSetpoint;
            return ret;
    
        }
        else if ((targetSetpoint.equals(INTAKE) || targetSetpoint.equals(HOME))
                && currentSetpoint.equals(PRE_CLIMB)) {

            return new ArmSetpoint[] {
                new ArmSetpoint(130,126).passShoulder().passElbow(),
                new ArmSetpoint(140, 87).passShoulder().passElbow(),
                targetSetpoint
            };
    
        } 
        else if (targetSetpoint.equals(PRE_CLIMB2) 
                && !(currentSetpoint.equals(PRE_CLIMB) || currentSetpoint.equals(PRE_CLIMB2))) {

            ArmSetpoint[] to1 = getPath(currentSetpoint, PRE_CLIMB);
            ArmSetpoint[] ret = Arrays.copyOf(to1, to1.length + 1);
            ret[ret.length - 1] = targetSetpoint;
            return ret;
    
        }
        else if (targetSetpoint.equals(INTAKE) && currentSetpoint.shoulder < 135) {
                
            return new ArmSetpoint[] {
                new ArmSetpoint(135, Math.max(currentSetpoint.elbow,0)).passShoulder().passElbow(),
                targetSetpoint
            };

        }
        else {

            return new ArmSetpoint[] {
                targetSetpoint
            };

        }
    }

}

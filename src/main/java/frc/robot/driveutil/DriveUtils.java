package frc.robot.driveutil;

import java.lang.Math;

public class DriveUtils{
    public static double signedPow(double base, int exp){
        double value = 0;
        
        if(base < 0 && exp%2==0){
            value = -Math.pow(base,exp);
        }
        else{
            value = Math.pow(base,exp);
        }

        return value;
    }

    public static double deadbandExponential(double spd, int exp, double deadband) {
        return Math.abs(spd)<deadband ? 0 : DriveUtils.signedPow(spd, exp) * (1 - deadband) + Math.signum(spd) * deadband;
    }

    public static double cheesyTurn(double spd, double turnRate) {
        if (spd == 0) {
          return turnRate;
    
        } else {
          return spd * turnRate;
        }
    
    }


}
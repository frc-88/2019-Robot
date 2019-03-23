package frc.robot.util;

import frc.robot.RobotMap;

public class ArmSetpoint {
    public double shoulder;
    public double elbow;
    public boolean passShoulder;
    public boolean passElbow;

    public ArmSetpoint(double shoulder, double elbow) {
        this.shoulder = shoulder;
        this.elbow = elbow;
        this.passShoulder = false;
        this.passElbow = false;
    }

    @Override
    public boolean equals(Object that) {
        if (!(that instanceof ArmSetpoint)) {
            return false;
        }

        ArmSetpoint thatSetpoint = (ArmSetpoint)that;

        return Math.abs(this.shoulder - thatSetpoint.shoulder) < RobotMap.ARM_TOLERANCE
                && Math.abs(this.elbow - thatSetpoint.elbow) < RobotMap.ARM_TOLERANCE;
    }

    public ArmSetpoint passShoulder() {
        this.passShoulder = true;
        return this;
    }

    public ArmSetpoint passElbow() {
        this.passElbow = true;
        return this;
    }
}
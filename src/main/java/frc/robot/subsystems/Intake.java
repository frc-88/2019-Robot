package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.*;

public class Intake extends Subsystem {
    TalonSRX rollerTalon;
    private final static int SLOTIDX = 0;
    private final static int TIMEOUTMS = 0;

    public Intake() {
        rollerTalon = new TalonSRX(RobotMap.INTAKE_ID);
        configTalon(rollerTalon);
    }

    private void configTalon(TalonSRX talon) {
        talon.configFactoryDefault();
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, SLOTIDX, TIMEOUTMS);
        talon.configNominalOutputForward(0.0, TIMEOUTMS);
        talon.configNominalOutputReverse(0.0, TIMEOUTMS);
        talon.configPeakOutputForward(+1.0, TIMEOUTMS);
        talon.configClosedloopRamp(0, TIMEOUTMS);
        talon.configPeakOutputReverse(-1.0, TIMEOUTMS);
        talon.configNeutralDeadband(0.01, TIMEOUTMS);
        talon.setNeutralMode(NeutralMode.Brake);
        talon.config_kP(SLOTIDX, 0, TIMEOUTMS);
        talon.config_kI(SLOTIDX, 0, TIMEOUTMS);
        talon.config_kD(SLOTIDX, 0, TIMEOUTMS);
        talon.config_kF(SLOTIDX, 0, TIMEOUTMS);
        talon.configMotionCruiseVelocity(50, TIMEOUTMS);
        talon.configMotionAcceleration(50, TIMEOUTMS);

    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());

    }

    public void set(double percentOutput){
        rollerTalon.set(ControlMode.PercentOutput, percentOutput);
    }
}
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class SAPG extends Subsystem{

    private TalonSRX sideMotor;
    private DoubleSolenoid deployPiston;
    private DoubleSolenoid grabPiston;

    public SAPG(){
        sideMotor = new TalonSRX(RobotMap.SAPG_MOTOR_ID);
        deployPiston = new DoubleSolenoid(RobotMap.SAPG_DEPLOY_FORWARD, RobotMap.SAPG_DEPLOY_REVERSE);
        grabPiston = new DoubleSolenoid(RobotMap.SAPG_GRAB_FORWARD, RobotMap.SAPG_GRAB_REVERSE);
    }

    public void configureTalon(){
        sideMotor.configFactoryDefault();
        sideMotor.setNeutralMode(NeutralMode.Coast);
        sideMotor.setInverted(false);
        sideMotor.configFeedbackNotContinuous(false, RobotMap.CAN_TIMEOUT);
        sideMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        sideMotor.configFeedbackNotContinuous(true, RobotMap.CAN_TIMEOUT);
        sideMotor.configForwardSoftLimitThreshold(feetToPotTicks(RobotMap.SAPG_SIDE_RANGE));
        sideMotor.configForwardSoftLimitEnable(true);
        sideMotor.configReverseSoftLimitThreshold(feetToPotTicks(-RobotMap.SAPG_SIDE_RANGE));
        sideMotor.configReverseSoftLimitEnable(true);
        sideMotor.configNeutralDeadband(0.01);
        sideMotor.enableVoltageCompensation(true);
        sideMotor.config_kP(0, 0);
        sideMotor.config_kI(0, 0);
        sideMotor.config_kD(0, 0);
        sideMotor.config_kF(0, 0);
    }

    public void openTheJaws(){
        grabPiston.set(Value.kForward);
    }

    public void moveToPosition(double position){
        sideMotor.set(ControlMode.Position, feetToPotTicks(position));
    }

    public void forwardPush(){
        deployPiston.set(Value.kForward);
    }

    public void reversePush(){
        deployPiston.set(Value.kReverse);
    }

    public void closeTheJaws(){
        grabPiston.set(Value.kReverse);
    }

    @Override
    protected void initDefaultCommand() {

    }

    public double potTicksToFeet(int potTicks) {
        return (potTicks  * RobotMap.SAPG_POT_SCALAR) + RobotMap.SAPG_POT_OFFSET; 
    }

    public int feetToPotTicks(double feet) {
        return (int)((feet - RobotMap.SAPG_POT_OFFSET) / RobotMap.SAPG_POT_SCALAR);
    }

}
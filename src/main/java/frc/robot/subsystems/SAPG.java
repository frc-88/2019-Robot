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
        sideMotor.configMotionCruiseVelocity(5);
        sideMotor.configMotionAcceleration(5);
        sideMotor.configFeedbackNotContinuous(true, RobotMap.CAN_TIMEOUT);
        sideMotor.configForwardSoftLimitThreshold(10);
        sideMotor.configForwardSoftLimitEnable(true);
        sideMotor.configReverseSoftLimitThreshold(-10);
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

    public void shiftTheSAPG(double position){
        sideMotor.set(ControlMode.Position, position);
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

}
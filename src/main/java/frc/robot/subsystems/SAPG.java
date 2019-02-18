package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.sapg.SAPGBasicControl;

public class SAPG extends Subsystem{
    private static final double Kp = 0;
    private static final double Ki = 0;
    private static final double Kd = 0;
    private static final double forwardLimit = 0;
    private static final double reverseLimit = 0;
    private static final double center = 0;

    private WPI_TalonSRX sideMotor;
    private DoubleSolenoid deployPiston;
    private DoubleSolenoid grabPiston;
    private PIDController sapgController;

    public void updateDashboard(){
        SmartDashboard.putNumber("SAPG:Position", getPosition());
        SmartDashboard.putNumber("SAPG:Voltage", sideMotor.getMotorOutputVoltage());
        SmartDashboard.putNumber("SAPG:Velocity", sideMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("SAPG:Current", sideMotor.getOutputCurrent());
    }

    public SAPG(){
        sideMotor = new WPI_TalonSRX(RobotMap.SAPG_MOTOR_ID);
        deployPiston = new DoubleSolenoid(RobotMap.SAPG_DEPLOY_PCM, RobotMap.SAPG_DEPLOY_FORWARD, RobotMap.SAPG_DEPLOY_REVERSE);
        grabPiston = new DoubleSolenoid(RobotMap.SAPG_GRAB_PCM, RobotMap.SAPG_GRAB_FORWARD, RobotMap.SAPG_GRAB_REVERSE);

        sapgController = new PIDController(Kp, Ki, Kd, Robot.m_limelight_back, sideMotor);
        sapgController.disable();

        initPreferences();
    }

    public void configureTalon(){
        sideMotor.configFactoryDefault();
        sideMotor.setNeutralMode(NeutralMode.Coast);
        sideMotor.setInverted(false);
        sideMotor.configFeedbackNotContinuous(false, RobotMap.CAN_TIMEOUT);
        sideMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        sideMotor.configMotionCruiseVelocity(5);
        sideMotor.configMotionAcceleration(5);
        sideMotor.config_kP(0, 0);
        sideMotor.config_kI(0, 0);
        sideMotor.config_kD(0, 0);
        sideMotor.config_kF(0 , 0);

        // sideMotor.configForwardSoftLimitThreshold(forwardLimit)
        // sideMotor.configReverseSoftLimitThreshold(reverseLimit)
        // sideMotor.configForwardSoftLimitEnable(true);
        // sideMotor.configReverseSoftLimitEnable(true);
    }

    private void initPreferences() {
        Preferences prefs = Preferences.getInstance();
    
        if (!prefs.containsKey("SAPGTestOutput")) { prefs.putDouble("SAPGTestOutput", 0.0); }
    }

    public void shiftTheSAPG(double position){
        sideMotor.set(ControlMode.MotionMagic, position);
    }
    
    public void set(double percentOutput){
        sideMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    public boolean atForwardLimit() {
        return sideMotor.getSelectedSensorPosition() > (forwardLimit - 30);
    }

    public boolean atReverseLimit() {
        return sideMotor.getSelectedSensorPosition() < (reverseLimit + 30);
    }

    public void openTheJaws(){
        grabPiston.set(Value.kForward);
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

    public void enableController() {
        sapgController.enable();
    }

    public void disableController() {
        sapgController.disable();
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new SAPGBasicControl());
    }

    public double getPosition(){
        return sideMotor.getSelectedSensorPosition();
    }
}
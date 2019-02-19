package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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

public class SAPG extends Subsystem{
    private static final Preferences prefs = Preferences.getInstance();

    private static final int forwardLimit = 660;
    private static final int reverseLimit = 320;
    private static final int center = reverseLimit + (forwardLimit - reverseLimit)/2;

    private WPI_TalonSRX sapgTalon;
    private DoubleSolenoid deployPiston;
    private DoubleSolenoid grabPiston;
    private PIDController sapgController;

    private double trackP;
    private double trackI;
    private double trackD;
    private double trackPeriod;

    public SAPG(){
        sapgTalon = new WPI_TalonSRX(RobotMap.SAPG_MOTOR_ID);
        deployPiston = new DoubleSolenoid(RobotMap.SAPG_DEPLOY_PCM, RobotMap.SAPG_DEPLOY_FORWARD, RobotMap.SAPG_DEPLOY_REVERSE);
        grabPiston = new DoubleSolenoid(RobotMap.SAPG_GRAB_PCM, RobotMap.SAPG_GRAB_FORWARD, RobotMap.SAPG_GRAB_REVERSE);

        configureTalon();
        initPreferences();
        fetchPreferences();

        sapgController = new PIDController(trackP, trackI, trackD, Robot.m_limelight_back, sapgTalon, trackPeriod);
    }

    private void configureTalon() {
        sapgTalon.configFactoryDefault();
        sapgTalon.setNeutralMode(NeutralMode.Coast);
        sapgTalon.setInverted(false);
        sapgTalon.configSelectedFeedbackSensor(FeedbackDevice.Analog);

        sapgTalon.configForwardSoftLimitThreshold(forwardLimit);
        sapgTalon.configReverseSoftLimitThreshold(reverseLimit);
        sapgTalon.configForwardSoftLimitEnable(true);
        sapgTalon.configReverseSoftLimitEnable(true);
    }

    private void initPreferences() {
        if (!prefs.containsKey("SAPG:Track_P")) { prefs.putDouble("SAPG:Track_P", 0.08); }
        if (!prefs.containsKey("SAPG:Track_I")) { prefs.putDouble("SAPG:Track_I", 0.0); }
        if (!prefs.containsKey("SAPG:Track_D")) { prefs.putDouble("SAPG:Track_D", 0.0); }
        if (!prefs.containsKey("SAPG:Track_Period")) { prefs.putDouble("SAPG:Track_Period", 0.01); }
    }

    private void fetchPreferences() {
        trackP = prefs.getDouble("SAPG:Track_P", 0.0);
        trackI = prefs.getDouble("SAPG:Track_I", 0.0);
        trackD = prefs.getDouble("SAPG:Track_D", 0.0);
        trackPeriod = prefs.getDouble("SAPG:Track_Period", 0.0);
    }


    public void set(double percentOutput){
        sapgTalon.set(ControlMode.PercentOutput, percentOutput);
    }

    public void openTheJaws(){
        grabPiston.set(Value.kForward);
    }

    public void closeTheJaws(){
        grabPiston.set(Value.kReverse);
    }

    public void forwardPush(){
        deployPiston.set(Value.kForward);
    }

    public void reversePush(){
        deployPiston.set(Value.kReverse);
    }

    public void enableTracking() {
        sapgController.setSetpoint(0);
        sapgController.enable();
    }

    public void disableTracking() {
        sapgController.disable();
    }

    public void updateDashboard(){
        SmartDashboard.putNumber("SAPG:Position", sapgTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("SAPG:Voltage", sapgTalon.getMotorOutputVoltage());
        SmartDashboard.putNumber("SAPG:Velocity", sapgTalon.getSelectedSensorVelocity());
        SmartDashboard.putNumber("SAPG:Current", sapgTalon.getOutputCurrent());
        SmartDashboard.putBoolean("SAPG:Tracking", sapgController.isEnabled());
    }

    @Override
    protected void initDefaultCommand() {
    }

}
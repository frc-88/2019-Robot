package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.sapg.SAPGDefault;
import frc.robot.util.SharpIR;

/**
 * 
 *      SAPG. Is it two
 *  Syllables or is it four?
 * Who cares? Watch it score!
 * 
 */
public class SAPG extends Subsystem {
    private static final Preferences prefs = Preferences.getInstance();

    private static final double DFT_VELOCITY_P = 0.0;
    private static final double DFT_VELOCITY_I = 0.0;
    private static final double DFT_VELOCITY_D = 0.0;
    private static final double DFT_VELOCITY_F = 0.5;
    private static final int DFT_MAX_SPEED = 15;
    private static final int MAIN_SLOT_IDX = 0;

    private TalonSRX sapgTalon;
    private DoubleSolenoid deployPiston;
    private DoubleSolenoid grabPiston;
    private SharpIR panelDetector;

    // Preferences with their default values
    private double velocityP = DFT_VELOCITY_P;
    private double velocityI = DFT_VELOCITY_I;
    private double velocityD = DFT_VELOCITY_D;
    private double velocityF = DFT_VELOCITY_F;
    private int maxSpeed = DFT_MAX_SPEED;
    private int forwardLimit = 1000;
    private int reverseLimit = 25;
    private double panelThreshold = 4.5;

    private int center = reverseLimit + (forwardLimit - reverseLimit) / 2;

    public SAPG() {
        sapgTalon = new TalonSRX(RobotMap.SAPG_MOTOR_ID);
        deployPiston = new DoubleSolenoid(RobotMap.SAPG_DEPLOY_PCM, RobotMap.SAPG_DEPLOY_FORWARD,
                RobotMap.SAPG_DEPLOY_REVERSE);
        grabPiston = new DoubleSolenoid(RobotMap.SAPG_GRAB_PCM, RobotMap.SAPG_GRAB_FORWARD, RobotMap.SAPG_GRAB_REVERSE);
        panelDetector = new SharpIR(RobotMap.SAPG_PANEL_IR_ID);

        initPreferences();
        fetchPreferences();

        open();
    }

    private void configureTalon() {
        sapgTalon.configFactoryDefault();
        sapgTalon.setNeutralMode(NeutralMode.Brake);
        sapgTalon.setInverted(false);
        sapgTalon.configSelectedFeedbackSensor(FeedbackDevice.Analog);

        sapgTalon.configNominalOutputForward(0.0);

        sapgTalon.configNominalOutputForward(0.0, RobotMap.CAN_TIMEOUT);
        sapgTalon.configNominalOutputReverse(0.0, RobotMap.CAN_TIMEOUT);
        sapgTalon.configPeakOutputForward(+1.0, RobotMap.CAN_TIMEOUT);
        sapgTalon.configClosedloopRamp(0, RobotMap.CAN_TIMEOUT);
        sapgTalon.configPeakOutputReverse(-1.0, RobotMap.CAN_TIMEOUT);

        sapgTalon.configForwardSoftLimitThreshold(forwardLimit);
        sapgTalon.configReverseSoftLimitThreshold(reverseLimit);
        sapgTalon.configForwardSoftLimitEnable(true);
        sapgTalon.configReverseSoftLimitEnable(true);

        sapgTalon.config_kP(MAIN_SLOT_IDX, velocityP, RobotMap.CAN_TIMEOUT);
        sapgTalon.config_kI(MAIN_SLOT_IDX, velocityI, RobotMap.CAN_TIMEOUT);
        sapgTalon.config_kD(MAIN_SLOT_IDX, velocityD, RobotMap.CAN_TIMEOUT);
        sapgTalon.config_kF(MAIN_SLOT_IDX, velocityF, RobotMap.CAN_TIMEOUT);
        sapgTalon.configMotionCruiseVelocity(maxSpeed, RobotMap.CAN_TIMEOUT);
        sapgTalon.configMotionAcceleration(2 * maxSpeed, RobotMap.CAN_TIMEOUT);
    }

    private void initPreferences() {
        if (!prefs.containsKey("SAPG:Velocity_P")) {
            prefs.putDouble("SAPG:Velocity_P", velocityP);
        }
        if (!prefs.containsKey("SAPG:Velocity_I")) {
            prefs.putDouble("SAPG:Velocity_I", velocityI);
        }
        if (!prefs.containsKey("SAPG:Velocity_D")) {
            prefs.putDouble("SAPG:Velocity_D", velocityD);
        }
        if (!prefs.containsKey("SAPG:Velocity_F")) {
            prefs.putDouble("SAPG:Velocity_D", velocityF);
        }
        if (!prefs.containsKey("SAPG:Max_Speed")) {
            prefs.putInt("SAPG:Max_Speed", maxSpeed);
        }
        if (!prefs.containsKey("SAPG:Forward_Limit")) {
            prefs.putInt("SAPG:Forward_Limit", forwardLimit);
        }
        if (!prefs.containsKey("SAPG:Reverse_Limit")) {
            prefs.putInt("SAPG:Reverse_Limit", reverseLimit);
        }
        if (!prefs.containsKey("SAPG:Panel_Threshold")) {
            prefs.putDouble("SAPG:Panel_Threshold", panelThreshold);
        }
    }

    public void fetchPreferences() {
        velocityP = prefs.getDouble("SAPG:Velocity_P", velocityP);
        velocityI = prefs.getDouble("SAPG:Velocity_I", velocityI);
        velocityD = prefs.getDouble("SAPG:Velocity_D", velocityD);
        velocityF = prefs.getDouble("SAPG:Velocity_D", velocityF);
        maxSpeed = prefs.getInt("SAPG:Max_Speed", maxSpeed);
        forwardLimit = prefs.getInt("SAPG:Forward_Limit", forwardLimit);
        reverseLimit = prefs.getInt("SAPG:Reverse_Limit", reverseLimit);
        panelThreshold = prefs.getDouble("SAPG:Panel_Threshold", panelThreshold);

        center = reverseLimit + (forwardLimit - reverseLimit) / 2;

        configureTalon();
    }

    public double getNormalizedPosition() {
        double normPos = ((double) (sapgTalon.getSelectedSensorPosition() - reverseLimit)
                / (double) (forwardLimit - reverseLimit)) * 2 - 1;

        if (normPos < -1) {
            normPos = -1;
        }

        if (normPos > 1) {
            normPos = 1;
        }
        return normPos;
    }

    private double dampNearLimits(double value) {
        return dampNearLimits(getNormalizedPosition(), value);
    }

    private double dampNearLimits(double position, double value) {
        // apply linear damping function near our limits
        if (Math.abs(position) > 0.8 && Math.signum(position) == Math.signum(value)) {
            value *= (1 - Math.abs(position)) * 5;
        }

        return value;
    }

    public void set(double percentOutput) {
        sapgTalon.set(ControlMode.PercentOutput, dampNearLimits(percentOutput));
    }

    public void goToPosition(int position) {
        if (position > reverseLimit && position < forwardLimit) {
            sapgTalon.set(ControlMode.MotionMagic, position);
        }
    }

    public void goToCenter() {
        goToPosition(center);
    }

    public void open() {
        grabPiston.set(Value.kForward);
    }

    public void close() {
        grabPiston.set(Value.kReverse);
    }

    public void deploy() {
        deployPiston.set(Value.kForward);
    }

    public void retract() {
        deployPiston.set(Value.kReverse);
    }

    public boolean hasPanel() {
        return (grabPiston.get() == Value.kForward) && (getPanelDistance() < panelThreshold);
    }

    public double getPanelDistance() {
        return panelDetector.getDistance();
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("SAPG:Position", sapgTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("SAPG:Velocity", sapgTalon.getSelectedSensorVelocity());
        SmartDashboard.putNumber("SAPG:PanelDistance", getPanelDistance());
        SmartDashboard.putBoolean("SAPG:HasPanel", hasPanel());

        // write prefs back to the dashboard
        SmartDashboard.putNumber("SAPG:Velocity_P", velocityP);
        SmartDashboard.putNumber("SAPG:Velocity_I", velocityI);
        SmartDashboard.putNumber("SAPG:Velocity_D", velocityD);
        SmartDashboard.putNumber("SAPG:Forward_Limit", forwardLimit);
        SmartDashboard.putNumber("SAPG:Reverse_Limit", reverseLimit);
        SmartDashboard.putNumber("SAPG:Panel_Threshold", panelThreshold);
    }

    @Override
    protected void initDefaultCommand() {
        //setDefaultCommand(new SAPGDefault());
    }

}

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
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
public class SAPG extends PIDSubsystem {
    private static final Preferences prefs = Preferences.getInstance();

    private static final double HORIZONTAL_FOV = 54.0;
    private static final double TRACK_ANGLE_THRESHOLD = (HORIZONTAL_FOV / 2) - 2;
    private static final double TRACK_DISTANCE_THRESHOLD = 12;
    private static final double TRACK_TICKS_THRESHOLD = 1000;
    private static final double TRACK_PID_DFT_P = 0.08;
    private static final double TRACK_PID_DFT_I = 0.0;
    private static final double TRACK_PID_DFT_D = 0.0;
    private static final double TRACK_PID_PERIOD = 0.01;

    private WPI_TalonSRX sapgTalon;
    private DoubleSolenoid deployPiston;
    private DoubleSolenoid grabPiston;
    private SharpIR panelDetector;

    // Preferences with their default values
    private double trackP = TRACK_PID_DFT_P;
    private double trackI = TRACK_PID_DFT_I;
    private double trackD = TRACK_PID_DFT_D;
    private int forwardLimit = 1010;
    private int reverseLimit = 80;
    private double panelThreshold = 4.5;
    private int center = reverseLimit + (forwardLimit - reverseLimit) / 2;
    private int home = center;
    private int ticksSinceTargetLost = 0;

    public SAPG() {
        super("SAPG", TRACK_PID_DFT_P, TRACK_PID_DFT_I, TRACK_PID_DFT_D, TRACK_PID_PERIOD);

        sapgTalon = new WPI_TalonSRX(RobotMap.SAPG_MOTOR_ID);
        deployPiston = new DoubleSolenoid(RobotMap.SAPG_DEPLOY_PCM, RobotMap.SAPG_DEPLOY_FORWARD, RobotMap.SAPG_DEPLOY_REVERSE);
        grabPiston = new DoubleSolenoid(RobotMap.SAPG_GRAB_PCM, RobotMap.SAPG_GRAB_FORWARD, RobotMap.SAPG_GRAB_REVERSE);
        panelDetector = new SharpIR(RobotMap.SAPG_PANEL_IR_ID);

        initPreferences();
        fetchPreferences();
        configureTalon();
        configurePIDController();

        home = sapgTalon.getSelectedSensorPosition();
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

    private void configurePIDController() {
        disable();
        getPIDController().reset();
        getPIDController().setPID(trackP, trackI, trackD);
        setOutputRange(-1, 1);
        setInputRange(-TRACK_ANGLE_THRESHOLD, TRACK_ANGLE_THRESHOLD);
        setAbsoluteTolerance(1);
        setSetpoint(0);
    }

    private void initPreferences() {
        if (!prefs.containsKey("SAPG:Track_P")) { prefs.putDouble("SAPG:Track_P", trackP); }
        if (!prefs.containsKey("SAPG:Track_I")) { prefs.putDouble("SAPG:Track_I", trackI); }
        if (!prefs.containsKey("SAPG:Track_D")) { prefs.putDouble("SAPG:Track_D", trackD); }
        if (!prefs.containsKey("SAPG:Forward_Limit")) { prefs.putDouble("SAPG:Forward_Limit", forwardLimit); }
        if (!prefs.containsKey("SAPG:Reverse_Limit")) { prefs.putDouble("SAPG:Reverse_Limit", reverseLimit); }
        if (!prefs.containsKey("SAPG:Panel_Threshold")) { prefs.putDouble("SAPG:Panel_Threshold", panelThreshold); }
    }

    public void fetchPreferences() {
        trackP = prefs.getDouble("SAPG:Track_P", trackP);
        trackI = prefs.getDouble("SAPG:Track_I", trackI);
        trackD = prefs.getDouble("SAPG:Track_D", trackD);
        forwardLimit = prefs.getInt("SAPG:Forward_Limit", forwardLimit);
        reverseLimit = prefs.getInt("SAPG:Reverse_Limit", reverseLimit);
        panelThreshold = prefs.getDouble("SAPG:Panel_Threshold", panelThreshold);

        configureTalon();
        configurePIDController();
    }

    private double getNormalizedPosition() {
        double normPos = ((double)(sapgTalon.getSelectedSensorPosition() - reverseLimit) / (double)(forwardLimit - reverseLimit)) * 2 - 1;
        
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

    public void openTheJaws() {
        grabPiston.set(Value.kForward);
    }

    public void closeTheJaws() {
        grabPiston.set(Value.kReverse);
    }

    public void forwardPush() {
        deployPiston.set(Value.kForward);
    }

    public void reversePush() {
        deployPiston.set(Value.kReverse);
    }

    public boolean hasPanel() {
        return (grabPiston.get() == Value.kForward) && (panelDetector.getDistance() < panelThreshold);
    }

    public double getPanelDistance() {
        return panelDetector.getDistance();
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("SAPG:Position", sapgTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("SAPG:Voltage", sapgTalon.getMotorOutputVoltage());
        SmartDashboard.putNumber("SAPG:Velocity", sapgTalon.getSelectedSensorVelocity());
        SmartDashboard.putNumber("SAPG:Current", sapgTalon.getOutputCurrent());
        SmartDashboard.putNumber("SAPG:PanelDistance", panelDetector.getDistance());
        SmartDashboard.putBoolean("SAPG:Tracking", getPIDController().isEnabled());
        SmartDashboard.putBoolean("SAPG:HasPanel", hasPanel());
        SmartDashboard.putBoolean("SAPG:OnTarget", onTarget());
        SmartDashboard.putNumber("SAPG:PanelDistance", getPanelDistance());

        // write prefs back to the dashboard
        SmartDashboard.putNumber("SAPG:Track_P", trackP);
        SmartDashboard.putNumber("SAPG:Track_I", trackI);
        SmartDashboard.putNumber("SAPG:Track_D", trackD);
        SmartDashboard.putNumber("SAPG:Forward_Limit", forwardLimit);
        SmartDashboard.putNumber("SAPG:Reverse_Limit", reverseLimit);
        SmartDashboard.putNumber("SAPG:Panel_Threshold", panelThreshold);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new SAPGDefault());
    }

    @Override
    protected double returnPIDInput() {
        double angle;
        // convert position to range 1 to -1, between limits
        double position = getNormalizedPosition();

        if (Robot.m_limelight_sapg.hasTarget()) {
            // If we have a target, track it
            ticksSinceTargetLost = 0;
            angle = Robot.m_limelight_sapg.getHorizontalOffsetAngle();
            // if angle offset is too large, hold current position
            if (Math.abs(angle) > TRACK_ANGLE_THRESHOLD) {
                angle = 0;
            }
            // if target is too close, hold current position
            if (Robot.m_limelight_sapg.getTargetDistance() < TRACK_DISTANCE_THRESHOLD) {
                angle = 0;
            }
        } else {
            // if we don't have a target,
            // and we haven't seen one in a while
            // TODO and we aren't facing a wall (forward facing IR sees short distance)
            // target center
            // TODO target home instead
            //
            if (ticksSinceTargetLost++ > TRACK_TICKS_THRESHOLD) {
                angle = position * TRACK_ANGLE_THRESHOLD;
            } else {
                angle = 0;
            }
        }

        //return dampNearLimits(position, angle);
        return angle;
    }

    @Override
    protected void usePIDOutput(double output) {
        set(output);
    }

}

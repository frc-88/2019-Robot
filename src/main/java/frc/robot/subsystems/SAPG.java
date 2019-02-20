package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.util.SharpIR;

public class SAPG extends Subsystem implements PIDSource {
    private static final Preferences prefs = Preferences.getInstance();

    private static final double HORIZONTAL_FOV = 54.0;
    private static final double TRACK_ANGLE_THRESHOLD = (HORIZONTAL_FOV / 2) - 2;
    private static final double TRACK_DISTANCE_THRESHOLD = 12;
    private static final double TRACK_TICKS_THRESHOLD = 1000;
    private static final double PANEL_THRESHOLD = 6.0;

    private WPI_TalonSRX sapgTalon;
    private DoubleSolenoid deployPiston;
    private DoubleSolenoid grabPiston;
    private PIDController sapgController;
    private SharpIR panelDetector;

    private double trackP = 0.08;
    private double trackI = 0.0;
    private double trackD = 0.0;
    private double trackPeriod = 0.01;
    private int forwardLimit = 1010;
    private int reverseLimit = 680;
    private int center = reverseLimit + (forwardLimit - reverseLimit) / 2;
    private int home = center;
    private int ticksSinceTargetLost = 0;

    public SAPG() {
        sapgTalon = new WPI_TalonSRX(RobotMap.SAPG_MOTOR_ID);
        configureTalon();
        deployPiston = new DoubleSolenoid(RobotMap.SAPG_DEPLOY_PCM, RobotMap.SAPG_DEPLOY_FORWARD,
                RobotMap.SAPG_DEPLOY_REVERSE);
        grabPiston = new DoubleSolenoid(RobotMap.SAPG_GRAB_PCM, RobotMap.SAPG_GRAB_FORWARD, RobotMap.SAPG_GRAB_REVERSE);
        panelDetector = new SharpIR(RobotMap.SAPG_PANEL_IR_ID);

        initPreferences();
        fetchPreferences();

        home = sapgTalon.getSelectedSensorPosition();

        sapgController = new PIDController(trackP, trackI, trackD, this, sapgTalon, trackPeriod);
        sapgController.setOutputRange(-1, 1);
        sapgController.setInputRange(-TRACK_ANGLE_THRESHOLD, TRACK_ANGLE_THRESHOLD);
        sapgController.setSetpoint(0);
        sapgController.disable();
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
        if (!prefs.containsKey("SAPG:Track_P")) {
            prefs.putDouble("SAPG:Track_P", trackP);
        }

        if (!prefs.containsKey("SAPG:Track_I")) {
            prefs.putDouble("SAPG:Track_I", trackI);
        }

        if (!prefs.containsKey("SAPG:Track_D")) {
            prefs.putDouble("SAPG:Track_D", trackD);
        }

        if (!prefs.containsKey("SAPG:Track_Period")) {
            prefs.putDouble("SAPG:Track_Period", trackPeriod);
        }

        if (!prefs.containsKey("SAPG:Forward_Limit")) {
            prefs.putDouble("SAPG:Forward_Limit", forwardLimit);
        }

        if (!prefs.containsKey("SAPG:Reverse_Limit")) {
            prefs.putDouble("SAPG:Reverse_Limit", reverseLimit);
        }
    }

    private void fetchPreferences() {
        trackP = prefs.getDouble("SAPG:Track_P", trackP);
        trackI = prefs.getDouble("SAPG:Track_I", trackI);
        trackD = prefs.getDouble("SAPG:Track_D", trackD);
        trackPeriod = prefs.getDouble("SAPG:Track_Period", trackPeriod);
        forwardLimit = prefs.getInt("SAPG:Forward_Limit", forwardLimit);
        reverseLimit = prefs.getInt("SAPG:Reverse_Limit", reverseLimit);
    }

    private double dampNearLimits(double value) {
        double position = ((sapgTalon.getSelectedSensorPosition() - reverseLimit) / (forwardLimit - reverseLimit)) * 2 - 1;

        return dampNearLimits(position, value);
    }

    private double dampNearLimits(double position, double value) {
        // apply linear damping function near our limits
        if (Math.abs(position) > 0.9) {
            value *= (1 - Math.abs(position)) * 10;
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

    public void enableTracking() {
        sapgController.setSetpoint(0);
        sapgController.enable();
    }

    public void disableTracking() {
        sapgController.disable();
    }

    public boolean hasPanel() {
        return panelDetector.getDistance() < PANEL_THRESHOLD;
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("SAPG:Position", sapgTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("SAPG:Voltage", sapgTalon.getMotorOutputVoltage());
        SmartDashboard.putNumber("SAPG:Velocity", sapgTalon.getSelectedSensorVelocity());
        SmartDashboard.putNumber("SAPG:Current", sapgTalon.getOutputCurrent());
        SmartDashboard.putNumber("SAPG:PanelDistance", panelDetector.getDistance());
        SmartDashboard.putBoolean("SAPG:Tracking", sapgController.isEnabled());
        SmartDashboard.putBoolean("SAPG:HasPanel", hasPanel());
    }

    // PIDSource overrides
    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        // PIDSourceType hard coded to kDisplacement
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
        double angle;
        // convert position to range 1 to -1, between limits
        double position = ((sapgTalon.getSelectedSensorPosition() - reverseLimit) / (forwardLimit - reverseLimit)) * 2
                - 1;

        if (!Robot.m_limelight_back.hasTarget()) {
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
        } else {
            ticksSinceTargetLost = 0;
            angle = Robot.m_limelight_back.getHorizontalOffsetAngle();
            // if angle offset is too large, hold current position
            if (Math.abs(angle) > TRACK_ANGLE_THRESHOLD) {
                angle = 0;
            }
            // if target is too close, hold current position
            if (Robot.m_limelight_back.getTargetDistanceByCameraTransform() < TRACK_DISTANCE_THRESHOLD) {
                angle = 0;
            }
        }

        return dampNearLimits(position, angle);
    }

    // Subsystem overrides
    @Override
    protected void initDefaultCommand() {
    }

}


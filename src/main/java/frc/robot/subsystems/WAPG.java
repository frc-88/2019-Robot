package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.util.SharpIR;

/**
 * 
 * it's wicked awesome 
 * and really good at grabbing 
 * yellow hatch panels
 * 
 */
public class WAPG extends Subsystem {
    private static final Preferences prefs = Preferences.getInstance();

    private DoubleSolenoid deployPiston;
    private DoubleSolenoid grabPiston;
    private SharpIR panelDetector;

    // Preferences with their default values
    private double panelThreshold = 4.5;

    public WAPG() {
        deployPiston = new DoubleSolenoid(RobotMap.WAPG_DEPLOY_PCM, RobotMap.WAPG_DEPLOY_FORWARD, RobotMap.WAPG_DEPLOY_REVERSE);
        grabPiston = new DoubleSolenoid(RobotMap.WAPG_GRAB_PCM, RobotMap.WAPG_GRAB_FORWARD, RobotMap.WAPG_GRAB_REVERSE);
        panelDetector = new SharpIR(RobotMap.WAPG_PANEL_IR_ID);

        initPreferences();
        fetchPreferences();

        open();
    }

    private void initPreferences() {
        if (!prefs.containsKey("WAPG:Panel_Threshold")) {
            prefs.putDouble("WAPG:Panel_Threshold", panelThreshold);
        }
    }

    public void fetchPreferences() {
        panelThreshold = prefs.getDouble("WAPG:Panel_Threshold", panelThreshold);
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
        SmartDashboard.putNumber("WAPG:PanelDistance", getPanelDistance());
        SmartDashboard.putBoolean("WAPG:HasPanel", hasPanel());

        // write prefs back to the dashboard
        SmartDashboard.putNumber("WAPG:Panel_Threshold", panelThreshold);
    }

    @Override
    protected void initDefaultCommand() {
        // no default command
    }

}

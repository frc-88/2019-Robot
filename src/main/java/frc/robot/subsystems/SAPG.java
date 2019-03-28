package frc.robot.subsystems;

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

    private DoubleSolenoid deployPiston;
    private DoubleSolenoid grabPiston;
    private SharpIR panelDetector;

    // Preferences with their default values
    private double panelThreshold = 4.5;

    public SAPG() {
        deployPiston = new DoubleSolenoid(RobotMap.SAPG_DEPLOY_PCM, RobotMap.SAPG_DEPLOY_FORWARD, RobotMap.SAPG_DEPLOY_REVERSE);
        grabPiston = new DoubleSolenoid(RobotMap.SAPG_GRAB_PCM, RobotMap.SAPG_GRAB_FORWARD, RobotMap.SAPG_GRAB_REVERSE);
        panelDetector = new SharpIR(RobotMap.SAPG_PANEL_IR_ID);

        initPreferences();
        fetchPreferences();

        open();
    }

    private void initPreferences() {
        if (!prefs.containsKey("SAPG:Panel_Threshold")) {
            prefs.putDouble("SAPG:Panel_Threshold", panelThreshold);
        }
    }

    public void fetchPreferences() {
        panelThreshold = prefs.getDouble("SAPG:Panel_Threshold", panelThreshold);
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
        SmartDashboard.putNumber("SAPG:PanelDistance", getPanelDistance());
        SmartDashboard.putBoolean("SAPG:HasPanel", hasPanel());

        // write prefs back to the dashboard
        SmartDashboard.putNumber("SAPG:Panel_Threshold", panelThreshold);
    }

    @Override
    protected void initDefaultCommand() {
        //setDefaultCommand(new SAPGDefault());
    }

}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * 
 * will this really be
 * the last iteration of
 * our panel grabber?
 * 
 */
public class LAPG extends Subsystem {
    private DoubleSolenoid deployPiston;
    private DoubleSolenoid grabPiston;
    private DoubleSolenoid grabNeutral;
    private DigitalInput grabSwitch;

    public LAPG() {
        deployPiston = new DoubleSolenoid(RobotMap.LAPG_DEPLOY_PCM, RobotMap.LAPG_DEPLOY_FORWARD, RobotMap.LAPG_DEPLOY_REVERSE);
        grabPiston = new DoubleSolenoid(RobotMap.LAPG_GRAB_PCM, RobotMap.LAPG_GRAB_FORWARD, RobotMap.LAPG_GRAB_REVERSE);
        grabNeutral = new DoubleSolenoid(RobotMap.LAPG_NEUTRAL_PCM, RobotMap.LAPG_NEUTRAL_FORWARD, RobotMap.LAPG_NEUTRAL_REVERSE);
        grabSwitch = new DigitalInput(RobotMap.LAPG_GRAB_SWITCH);

        open();
        neutral();
    }

    public void open() {
        grabPiston.set(Value.kForward);
    }

    public void close() {
        grabPiston.set(Value.kReverse);
    }

    public void off() {
        grabPiston.set(Value.kReverse);
    }

    public void neutral() {
        grabNeutral.set(Value.kForward);
    }

    public void active() {
        grabNeutral.set(Value.kReverse);
    }

    public void deploy() {
        deployPiston.set(Value.kForward);
    }

    public void retract() {
        deployPiston.set(Value.kReverse);
    }

    public boolean getSwitch() {
        return grabSwitch.get();
    }

    public void updateDashboard() {
        SmartDashboard.putString("LAPG:grab", grabPiston.get().toString());
        SmartDashboard.putString("LAPG:deploy", deployPiston.get().toString());
        SmartDashboard.putString("LAPG:neutral", grabNeutral.get().toString());
        SmartDashboard.putBoolean("LAPG:switch", grabSwitch.get());
    }

    @Override
    protected void initDefaultCommand() {
        // no default command
    }

}

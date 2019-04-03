package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * 
 * it's wicked awesome 
 * and really good at grabbing 
 * yellow hatch panels
 * 
 */
public class WAPG extends Subsystem {
    private DoubleSolenoid deployPiston;
    private DoubleSolenoid grabPiston;

    public WAPG() {
        deployPiston = new DoubleSolenoid(RobotMap.WAPG_DEPLOY_PCM, RobotMap.WAPG_DEPLOY_FORWARD, RobotMap.WAPG_DEPLOY_REVERSE);
        grabPiston = new DoubleSolenoid(RobotMap.WAPG_GRAB_PCM, RobotMap.WAPG_GRAB_FORWARD, RobotMap.WAPG_GRAB_REVERSE);

        open();
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

    @Override
    protected void initDefaultCommand() {
        // no default command
    }

}

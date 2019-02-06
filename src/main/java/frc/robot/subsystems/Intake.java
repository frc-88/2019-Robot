package frc.robot.subsystems;
import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.*;
public class Intake extends Subsystem{
 TalonSRX rollerTalon;
 private final static int SLOTIDX = 0;
private final static int TIMEOUTMS = 0;
    public Intake(){
        rollerTalon= new TalonSRX(RobotMap.INTAKE_ID);

    }

    private void configTalonCommon(TalonSRX talon){
        talon.configFactoryDefault();
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, SLOTIDX, TIMEOUTMS);

    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    
      }
}
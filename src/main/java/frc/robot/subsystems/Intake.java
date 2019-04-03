package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.BufferedWriter;
import java.io.IOException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.RobotMap;
import frc.robot.commands.intake.IntakeDefault;
import frc.robot.util.SharpIR;

/**
 * 
 * Cargo Everywhere
 * Time to put them back in bins
 * Get me a panel!
 * 
 */
public class Intake extends Subsystem {
    TalonSRX rollerTalon;
    SharpIR intakeSensor1, intakeSensor2;
    private final static int SLOTIDX = 0;
    private final static int TIMEOUTMS = 0;
    private boolean objectSeen = false;

    public Intake() {
        rollerTalon = new TalonSRX(RobotMap.INTAKE_ID);
        intakeSensor1 = new SharpIR(RobotMap.INTAKE_IR1_ID);
        intakeSensor2 = new SharpIR(RobotMap.INTAKE_IR2_ID);

        configTalon(rollerTalon);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Cargo Distance 1", intakeSensor1.getDistance());
        SmartDashboard.putNumber("Cargo Distance 2", intakeSensor2.getDistance());
        SmartDashboard.putNumber("Cargo Distance Avg", getAverageDistance());
        SmartDashboard.putBoolean("Has Cargo", hasCargo());
    }

    public void logDistanceData(BufferedWriter file) {
        double dist1 = intakeSensor1.getDistance();
        double dist2 = intakeSensor2.getDistance();

        try {
            if (hasCargo()) {
                if (!objectSeen) {
                    file.write("object begin");
                    file.newLine();
                    objectSeen = true;
                }
                file.write(String.format("%f,%f%n", dist1, dist2));
                file.newLine();
            } else {
                if (objectSeen) {
                    file.write("object end");
                    file.newLine();
                    objectSeen = false;
                }
            }
        } catch (IOException ioe) {
            ioe.printStackTrace();
        }
    }

    public boolean hasCargo() {
        return getAverageDistance() < RobotMap.INTAKE_HAS_CARGO;
        //return intakeSensor1.getDistance() < RobotMap.INTAKE_HAS_CARGO * 2;
    }

    public double getAverageDistance() {
        return ((intakeSensor1.getDistance() + intakeSensor2.getDistance()) / 2.0);
    }

    private void configTalon(TalonSRX talon) {
        talon.configFactoryDefault();
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, SLOTIDX, TIMEOUTMS);
        talon.configNominalOutputForward(0.0, TIMEOUTMS);
        talon.configNominalOutputReverse(0.0, TIMEOUTMS);
        talon.configPeakOutputForward(+1.0, TIMEOUTMS);
        talon.configClosedloopRamp(0, TIMEOUTMS);
        talon.configPeakOutputReverse(-1.0, TIMEOUTMS);
        talon.configNeutralDeadband(0.01, TIMEOUTMS);
        talon.setNeutralMode(NeutralMode.Brake);
        talon.config_kP(SLOTIDX, 0, TIMEOUTMS);
        talon.config_kI(SLOTIDX, 0, TIMEOUTMS);
        talon.config_kD(SLOTIDX, 0, TIMEOUTMS);
        talon.config_kF(SLOTIDX, 0, TIMEOUTMS);
        talon.configMotionCruiseVelocity(50, TIMEOUTMS);
        talon.configMotionAcceleration(50, TIMEOUTMS);
    }

    public void initDefaultCommand() {
        setDefaultCommand(new IntakeDefault());
    }

    public void set(double percentOutput) {
        rollerTalon.set(ControlMode.PercentOutput, percentOutput);
    }
}
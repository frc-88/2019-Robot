/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner6;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.test.DriveConstantVoltage;
import frc.robot.commands.drive.test.DriveIncrease;
import frc.robot.commands.drive.test.DriveIncreaseVel;
import frc.robot.driveutil.DriveConfiguration;
import frc.robot.driveutil.TJDriveModule;
import frc.robot.util.TJPIDController;
import frc.robot.util.transmission.ShiftingTransmission;
import frc.robot.util.transmission.TalonGrayhill;
import frc.robot.util.transmission.Vex775Pro;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Drive extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private TJDriveModule leftDrive, rightDrive;
    private ShiftingTransmission transmission;
    private DriveConfiguration driveConfiguration;
    private TJPIDController velocityController;

    private DoubleSolenoid leftShifter;
    private DoubleSolenoid rightShifter;
    private AHRS navX;

    private double maxSpeed;

    private NetworkTableEntry sbLeftDriveMode;
    private NetworkTableEntry sbRightDriveMode;
    private NetworkTableEntry sbLeftVoltage;
    private NetworkTableEntry sbRightVoltage;
    private NetworkTableEntry sbLeftCurrent;
    private NetworkTableEntry sbRightCurrent;
    private NetworkTableEntry sbLeftSpeed;
    private NetworkTableEntry sbRightSpeed;
    private NetworkTableEntry sbAngularSpeed;
    private NetworkTableEntry sbYaw;
    private NetworkTableEntry sbTestDriveVoltage;
    private NetworkTableEntry sbLeftPredictedCurrentDraw;
    private NetworkTableEntry sbRightPredictedCurrentDraw;
    private NetworkTableEntry sbCurrentLimit;
    private NetworkTableEntry sbLeftCommandedSpeed;
    private NetworkTableEntry sbRightCommandedSpeed;
    private NetworkTableEntry sbMaxAccel;
    private NetworkTableEntry sbVelKp;
    private NetworkTableEntry sbVelKi;
    private NetworkTableEntry sbVelKd;
    private NetworkTableEntry sbVelIZone;
    private NetworkTableEntry sbVelIMax;

    private DriveConstantVoltage constantDriveTestCommand;
    private double currentLimit = RobotMap.DRIVE_CURRENT_LIMIT;
    private double maxAccel = RobotMap.MAX_ACCEL_LOW;

    boolean resetFromShift = false;

    private double leftCommandedSpeed;
    private double rightCommandedSpeed;
    private double joystickSpeed;


    public Drive() {
        transmission = new ShiftingTransmission(
                new Vex775Pro(), RobotMap.NUM_DRIVE_MOTORS_PER_SIDE, new TalonGrayhill(),
                RobotMap.LOW_DRIVE_RATIO, RobotMap.HIGH_DRIVE_RATIO, RobotMap.DRIVE_SENSOR_RATIO,
                RobotMap.DRIVE_LOW_STATIC_FRICTION_VOLTAGE, RobotMap.DRIVE_HIGH_STATIC_FRICTION_VOLTAGE,
                RobotMap.DRIVE_LOW_EFFICIENCY, RobotMap.DRIVE_HIGH_EFFICIENCY);

        velocityController = new TJPIDController(
            RobotMap.DRIVE_VEL_LOW_KP, 
            RobotMap.DRIVE_VEL_LOW_KI, 
            RobotMap.DRIVE_VEL_LOW_KD, 
            RobotMap.DRIVE_VEL_LOW_IZONE, 
            RobotMap.DRIVE_VEL_LOW_IMAX);
        transmission.setVelocityPID(velocityController);

        driveConfiguration = new DriveConfiguration();

        leftDrive = new TJDriveModule(driveConfiguration.left, transmission);
        rightDrive = new TJDriveModule(driveConfiguration.right, transmission);

        leftShifter = new DoubleSolenoid(RobotMap.SHIFTER_LEFT_PCM, RobotMap.SHIFTER_LEFT_OUT, RobotMap.SHIFTER_LEFT_IN);
        rightShifter = new DoubleSolenoid(RobotMap.SHIFTER_RIGHT_PCM, RobotMap.SHIFTER_RIGHT_OUT, RobotMap.SHIFTER_RIGHT_IN);
        navX = new AHRS(SPI.Port.kMXP);

        transmission.shiftToLow();
        maxSpeed = transmission.getHighMaxOutputSpeed(RobotMap.MAX_DRIVE_VOLTAGE);

        //addBadlogsTopics();
    }


    public void configureShuffleboard() {
        // Talon status info
        sbLeftDriveMode = Shuffleboard.getTab("Drivetrain").add("Left Mode", "").getEntry();
        sbRightDriveMode = Shuffleboard.getTab("Drivetrain").add("Right Mode", "").getEntry();

        // Power info
        sbLeftVoltage = Shuffleboard.getTab("Drivetrain").add("Left Voltage", 0).getEntry();
        sbRightVoltage = Shuffleboard.getTab("Drivetrain").add("Right Voltage", 0).getEntry();
        sbLeftCurrent = Shuffleboard.getTab("Drivetrain").add("Left Current", 0).getEntry();
        sbRightCurrent = Shuffleboard.getTab("Drivetrain").add("Right Current", 0).getEntry();

        // Get encoder info
        sbLeftSpeed = Shuffleboard.getTab("Drivetrain").add("Left Speed", 0).getEntry();
        sbRightSpeed = Shuffleboard.getTab("Drivetrain").add("Right Speed",0).getEntry();

        // Get NavX info
        sbAngularSpeed = Shuffleboard.getTab("Drivetrain").add("Angular Speed", 0).getEntry();
        sbYaw = Shuffleboard.getTab("Drivetrain").add("Yaw", 0).getEntry();

        //commanded speeds
        sbLeftCommandedSpeed = Shuffleboard.getTab("Drivetrain").add("Left Cmmd Spd", 0).getEntry();
        sbRightCommandedSpeed = Shuffleboard.getTab("Drivetrain").add("Right cmmd Spd", 0).getEntry();

        // Test constant voltage command
       constantDriveTestCommand = new DriveConstantVoltage(0);
       Shuffleboard.getTab("TestDrive").add("Constant Drive", constantDriveTestCommand);
       Shuffleboard.getTab("TestDrive").add("Increasing Drive", new DriveIncrease());
       Shuffleboard.getTab("TestDrive").add("Increasing Drive Vel", new DriveIncreaseVel());
       sbTestDriveVoltage = Shuffleboard.getTab("TestDrive").add("Constant Voltage", 0.).getEntry();
       sbLeftPredictedCurrentDraw = Shuffleboard.getTab("TestDrive").add("Left Pred Current", 0).getEntry();
       sbRightPredictedCurrentDraw = Shuffleboard.getTab("TestDrive").add("Right Pred Current", 0).getEntry();
       sbCurrentLimit = Shuffleboard.getTab("TestDrive").add("Current Limit", currentLimit).getEntry();
       sbMaxAccel = Shuffleboard.getTab("TestDrive").add("Max Accel", maxAccel).getEntry();

        // Velocity PID tuning
        sbVelKp = Shuffleboard.getTab("DriveVel").add("kP", RobotMap.DRIVE_VEL_LOW_KP).getEntry();
        sbVelKi = Shuffleboard.getTab("DriveVel").add("kI", RobotMap.DRIVE_VEL_LOW_KI).getEntry();
        sbVelKd = Shuffleboard.getTab("DriveVel").add("kD", RobotMap.DRIVE_VEL_LOW_KD).getEntry();
        sbVelIZone = Shuffleboard.getTab("DriveVel").add("iZone", RobotMap.DRIVE_VEL_LOW_IZONE).getEntry();
        sbVelIMax = Shuffleboard.getTab("DriveVel").add("iMax", RobotMap.DRIVE_VEL_LOW_IMAX).getEntry();

    }

    public void updateShuffleboard() {
        // Talon status info
        sbLeftDriveMode.setString(leftDrive.getControlMode().toString());
        sbRightDriveMode.setString(rightDrive.getControlMode().toString());

        // Power info
        sbLeftVoltage.setDouble(leftDrive.getMotorOutputVoltage());
        sbRightVoltage.setDouble(rightDrive.getMotorOutputVoltage());
        sbLeftCurrent.setDouble(leftDrive.getTotalCurrent());
        sbRightCurrent.setDouble(rightDrive.getTotalCurrent());

        // Get encoder info
        sbLeftSpeed.setDouble(getLeftSpeed());
        sbRightSpeed.setDouble(getRightSpeed());

        // Get NavX info
        sbAngularSpeed.setDouble(getAngularVelocity());
        sbYaw.setDouble(getAngle());

        // Commanded Speeds
        sbLeftCommandedSpeed.setDouble(leftCommandedSpeed);
        sbRightCommandedSpeed.setDouble(rightCommandedSpeed);

        // Test Constant Voltage Command
        constantDriveTestCommand.setVoltage(sbTestDriveVoltage.getDouble(0));
        sbLeftPredictedCurrentDraw.setDouble(transmission.getCurrentDraw(leftDrive.getMotorOutputVoltage(), leftDrive.getSelectedSensorVelocity()));
        sbRightPredictedCurrentDraw.setDouble(transmission.getCurrentDraw(rightDrive.getMotorOutputVoltage(), rightDrive.getSelectedSensorVelocity()));
        currentLimit = sbCurrentLimit.getDouble(RobotMap.DRIVE_CURRENT_LIMIT);
        if (resetFromShift) {
            sbMaxAccel.setDouble(maxAccel);
        } else {
            maxAccel = sbMaxAccel.getDouble(RobotMap.MAX_ACCEL_LOW);
        }

        // Velocity PID tuning
        if (resetFromShift) {
            sbVelKp.setDouble(velocityController.getKP());
            sbVelKi.setDouble(velocityController.getKI());
            sbVelKd.setDouble(velocityController.getKD());
            sbVelIZone.setDouble(velocityController.getIZone());
            sbVelIMax.setDouble(velocityController.getIMax());
        } else {
            velocityController.setKP(sbVelKp.getDouble(RobotMap.DRIVE_VEL_LOW_KP));
            velocityController.setKI(sbVelKi.getDouble(RobotMap.DRIVE_VEL_LOW_KI));
            velocityController.setKD(sbVelKd.getDouble(RobotMap.DRIVE_VEL_LOW_KD));
            velocityController.setIZone(sbVelIZone.getDouble(RobotMap.DRIVE_VEL_LOW_IZONE));
            velocityController.setIMax(sbVelIMax.getDouble(RobotMap.DRIVE_VEL_LOW_IMAX));
        }


        resetFromShift = false;
    }

    public void basicDrive(double leftSpeed, double rightSpeed) {
        System.out.println(leftSpeed + " " + rightSpeed);
        leftDrive.set(ControlMode.PercentOutput, leftSpeed);
        rightDrive.set(ControlMode.PercentOutput, rightSpeed);

    }

    /**
     * 
     * Commands the drivetrain to the given velocities (in fps) while
     * proactively limiting current draw.
     */
    public void basicDriveLimited(double leftVelocity, double rightVelocity) {
        leftCommandedSpeed = leftVelocity;
        rightCommandedSpeed = rightVelocity;
        leftDrive.setVelocityCurrentLimited(leftVelocity, currentLimit/2);
        rightDrive.setVelocityCurrentLimited(rightVelocity, currentLimit/2);
    }

    /**
     * Arcade drive function for teleop control.
     * 
     * Parameters:
     *  @param speed The forwards/backwards speed on a scale from -1 to 1
     *  @param turnRate The rate to turn at on a scale from 
     *                  -1 (counterclockwise) to 1 (clockwise)
     */
    public void arcadeDrive(double speed, double turn) {
        // speed *= maxSpeed;
        // turn *= maxSpeed;
        // joystickSpeed = speed;
        // speed = limitAcceleration(speed);
        // double leftSpeed = (speed + turn);
        // double rightSpeed = (speed - turn);
        // basicDriveLimited(leftSpeed, rightSpeed);
        basicDrive(speed + turn, speed - turn);
    }

    public double limitAcceleration(double speed){
        if (speed - getStraightSpeed() > 0){
            double vel = getStraightSpeed() + maxAccel;
            if(speed<vel){
                return speed;
            }
            else{
                return vel;
            }
        }
        else{
            double vel = getStraightSpeed() - maxAccel;
            if(speed>vel){
                return speed;
            }
            else{
                return vel;
            }

        }


    }

    /*

Old versions of arcade drive:

    public void arcadeDrive(double speed, double turn) {
        double leftSpeed = speed + turn;
        double rightSpeed = speed - turn;
        basicDrive(leftSpeed, rightSpeed);

    }
    */

    public void resetVelocityPID() {
        velocityController.reset();
    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new ArcadeDrive());

    }

    public void setGearFromButton() {
        if (Robot.m_oi.getHighGearButton()) {
            this.shiftToLow();

        } else {
            this.shiftToHigh();
        }
        maxSpeed = transmission.getMaxOutputSpeed(RobotMap.MAX_DRIVE_VOLTAGE);
    }

    public void autoshift() {
        if (Robot.m_oi.getForceLowGearButton()) {
            this.shiftToLow();
            maxSpeed = transmission.getLowMaxOutputSpeed(RobotMap.MAX_DRIVE_VOLTAGE);
        } else {
            if(isInHighGear()&&Math.abs(getStraightSpeed())<=RobotMap.SHIFT_INTO_LOW_GEAR){
                shiftToLow();
            }
            else
                if(!isInHighGear()&&Math.abs(getStraightSpeed())>=RobotMap.SHIFT_INTO_HIGH_GEAR
                        && Math.abs(joystickSpeed) > RobotMap.COMMANDED_STOP_SPEED){
                    shiftToHigh();
                }
            else
                if(isInHighGear()&&Math.abs(getStraightSpeed())>=RobotMap.SHIFT_INTO_LOW_GEAR_STOP&&Math.abs(joystickSpeed)<=RobotMap.COMMANDED_STOP_SPEED){
                    shiftToLow();
            }
            maxSpeed = transmission.getHighMaxOutputSpeed(RobotMap.MAX_DRIVE_VOLTAGE);
        }
    }

    public void shiftToLow() {
        leftShifter.set(Value.kReverse);
        rightShifter.set(Value.kReverse);
        transmission.shiftToLow();
        velocityController.setKP(RobotMap.DRIVE_VEL_LOW_KP);
        velocityController.setKI(RobotMap.DRIVE_VEL_LOW_KI);
        velocityController.setKD(RobotMap.DRIVE_VEL_LOW_KD);
        velocityController.setIZone(RobotMap.DRIVE_VEL_LOW_IZONE);
        velocityController.setIMax(RobotMap.DRIVE_VEL_LOW_IMAX);
        maxAccel = RobotMap.MAX_ACCEL_LOW;
        resetFromShift = true;
    }

    public void shiftToHigh() {
        leftShifter.set(Value.kForward);
        rightShifter.set(Value.kForward);
        transmission.shiftToHigh();
        velocityController.setKP(RobotMap.DRIVE_VEL_HIGH_KP);
        velocityController.setKI(RobotMap.DRIVE_VEL_HIGH_KI);
        velocityController.setKD(RobotMap.DRIVE_VEL_HIGH_KD);
        velocityController.setIZone(RobotMap.DRIVE_VEL_HIGH_IZONE);
        velocityController.setIMax(RobotMap.DRIVE_VEL_HIGH_IMAX);
        maxAccel = RobotMap.MAX_ACCEL_HIGH;
        resetFromShift = true;
    }

    public boolean isInHighGear() {
        return transmission.isInHighGear();
    }

    public double getLeftPosition() {
        return leftDrive.getScaledSensorPosition();
    }

    public double getRightPosition() {
        return rightDrive.getScaledSensorPosition();
    }

    public double getLeftSpeed() {
        return leftDrive.getScaledSensorVelocity();
    }

    public double getRightSpeed() {
        return rightDrive.getScaledSensorVelocity();
    }

    public double getStraightSpeed() {
        return (getLeftSpeed() + getRightSpeed()) / 2;
    }

    public double getAngularVelocity() {
        return Math.toDegrees(navX.getRate());
    }

    public double getAngle() {
        return navX.getYaw();
    }

    public void resetYaw() {
        navX.reset();
    }

}

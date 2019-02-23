/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import java.util.List;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.test.DriveConstantVoltage;
import frc.robot.commands.drive.test.DriveIncrease;
import frc.robot.commands.drive.test.DriveIncreaseVel;
import frc.robot.driveutil.DriveConfiguration;
import frc.robot.driveutil.DriveUtils;
import frc.robot.driveutil.TJDriveModule;
import frc.robot.driveutil.TJDriveMotionPoint;
import frc.robot.util.TJPIDController;
import frc.robot.util.transmission.CTREMagEncoder;
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
    private ShiftingTransmission leftTransmission, rightTransmission;
    private DriveConfiguration driveConfiguration;
    private TJPIDController leftVelocityController, rightVelocityController;

    private DoubleSolenoid leftShifter, rightShifter;

    private double maxSpeed;

    private NetworkTableEntry sbLeftDriveMode;
    private NetworkTableEntry sbRightDriveMode;
    private NetworkTableEntry sbLeftVoltage;
    private NetworkTableEntry sbRightVoltage;
    private NetworkTableEntry sbLeftCurrent;
    private NetworkTableEntry sbRightCurrent;
    private NetworkTableEntry sbLeftSpeed;
    private NetworkTableEntry sbRightSpeed;
    private NetworkTableEntry sbLeftPos;
    private NetworkTableEntry sbRightPos;
    private NetworkTableEntry sbTestDriveVoltage;
    private NetworkTableEntry sbLeftPredictedCurrentDraw;
    private NetworkTableEntry sbRightPredictedCurrentDraw;
    private NetworkTableEntry sbCurrentLimit;
    private NetworkTableEntry sbLeftCommandedSpeed;
    private NetworkTableEntry sbRightCommandedSpeed;
    private NetworkTableEntry sbVelKp;
    private NetworkTableEntry sbVelKi;
    private NetworkTableEntry sbVelKd;
    private NetworkTableEntry sbVelIZone;
    private NetworkTableEntry sbVelIMax;
    private NetworkTableEntry sbMoProLeftFF;
    private NetworkTableEntry sbMoProRightFF;
    private NetworkTableEntry sbMoProLeftPos;
    private NetworkTableEntry sbMoProRightPos;
    private NetworkTableEntry sbMoProLeftVel;
    private NetworkTableEntry sbMoProRightVel;
    private NetworkTableEntry sbMoProP;
    private NetworkTableEntry sbMoProI;
    private NetworkTableEntry sbMoProD;

    private DriveConstantVoltage constantDriveTestCommand;
    private double currentLimit = RobotMap.DRIVE_CURRENT_LIMIT;

    private double moProKP;
    private double moProKI;
    private double moProKD;

    boolean resetFromShift = false;

    private double leftCommandedSpeed;
    private double rightCommandedSpeed;
    private double joystickSpeed;

    public Drive() {
        leftTransmission = new ShiftingTransmission(new Vex775Pro(), RobotMap.NUM_DRIVE_MOTORS_PER_SIDE,
                new CTREMagEncoder(), RobotMap.LOW_DRIVE_RATIO, RobotMap.HIGH_DRIVE_RATIO, RobotMap.DRIVE_SENSOR_RATIO,
                RobotMap.DRIVE_LOW_STATIC_FRICTION_VOLTAGE, RobotMap.DRIVE_HIGH_STATIC_FRICTION_VOLTAGE,
                RobotMap.DRIVE_LEFT_LOW_EFFICIENCY, RobotMap.DRIVE_LEFT_HIGH_EFFICIENCY);
        rightTransmission = new ShiftingTransmission(new Vex775Pro(), RobotMap.NUM_DRIVE_MOTORS_PER_SIDE,
                new CTREMagEncoder(), RobotMap.LOW_DRIVE_RATIO, RobotMap.HIGH_DRIVE_RATIO, RobotMap.DRIVE_SENSOR_RATIO,
                RobotMap.DRIVE_LOW_STATIC_FRICTION_VOLTAGE, RobotMap.DRIVE_HIGH_STATIC_FRICTION_VOLTAGE,
                RobotMap.DRIVE_RIGHT_LOW_EFFICIENCY, RobotMap.DRIVE_RIGHT_HIGH_EFFICIENCY);

        leftVelocityController = new TJPIDController(RobotMap.DRIVE_VEL_LOW_KP, RobotMap.DRIVE_VEL_LOW_KI,
                RobotMap.DRIVE_VEL_LOW_KD, RobotMap.DRIVE_VEL_LOW_IZONE, RobotMap.DRIVE_VEL_LOW_IMAX);
        rightVelocityController = new TJPIDController(RobotMap.DRIVE_VEL_LOW_KP, RobotMap.DRIVE_VEL_LOW_KI,
                RobotMap.DRIVE_VEL_LOW_KD, RobotMap.DRIVE_VEL_LOW_IZONE, RobotMap.DRIVE_VEL_LOW_IMAX);
        leftTransmission.setVelocityPID(leftVelocityController);
        rightTransmission.setVelocityPID(rightVelocityController);

        driveConfiguration = new DriveConfiguration();

        moProKP = driveConfiguration.left.masterConfiguration.slot0.kP;
        moProKI = driveConfiguration.left.masterConfiguration.slot0.kI;
        moProKD = driveConfiguration.left.masterConfiguration.slot0.kD;

        leftDrive = new TJDriveModule(driveConfiguration.left, leftTransmission);
        rightDrive = new TJDriveModule(driveConfiguration.right, rightTransmission);

        leftShifter = new DoubleSolenoid(RobotMap.SHIFTER_LEFT_PCM, RobotMap.SHIFTER_LEFT_OUT,
                RobotMap.SHIFTER_LEFT_IN);
        rightShifter = new DoubleSolenoid(RobotMap.SHIFTER_RIGHT_PCM, RobotMap.SHIFTER_RIGHT_OUT,
                RobotMap.SHIFTER_RIGHT_IN);

        leftTransmission.shiftToLow();
        rightTransmission.shiftToLow();
        maxSpeed = RobotMap.MAX_SPEED_LOW;
    }

    public void configureShuffleboard() {
        // Talon status info
        sbLeftDriveMode = Shuffleboard.getTab("Drivetrain").add("Left Mode", "").getEntry();
        sbRightDriveMode = Shuffleboard.getTab("Drivetrain").add("Right Mode", "").getEntry();

        // Power info
        sbLeftVoltage = Shuffleboard.getTab("Drivetrain").add("Left Voltage", 0).getEntry();
        Robot.dashboardScheduler.addFunction(() -> sbLeftVoltage.setDouble(leftDrive.getMotorOutputVoltage()));
        sbRightVoltage = Shuffleboard.getTab("Drivetrain").add("Right Voltage", 0).getEntry();
        Robot.dashboardScheduler.addFunction(() -> sbRightVoltage.setDouble(rightDrive.getMotorOutputVoltage()));
        sbLeftCurrent = Shuffleboard.getTab("Drivetrain").add("Left Current", 0).getEntry();
        Robot.dashboardScheduler.addFunction(() -> sbLeftCurrent.setDouble(leftDrive.getTotalCurrent()));
        sbRightCurrent = Shuffleboard.getTab("Drivetrain").add("Right Current", 0).getEntry();
        Robot.dashboardScheduler.addFunction(() -> sbRightCurrent.setDouble(rightDrive.getTotalCurrent()));

        // Get encoder info
        sbLeftSpeed = Shuffleboard.getTab("Drivetrain").add("Left Speed", 0).getEntry();
        Robot.dashboardScheduler.addFunction(() -> sbLeftSpeed.setDouble(getLeftSpeed()));
        sbRightSpeed = Shuffleboard.getTab("Drivetrain").add("Right Speed", 0).getEntry();
        Robot.dashboardScheduler.addFunction(() -> sbRightSpeed.setDouble(getRightSpeed()));
        sbLeftPos = Shuffleboard.getTab("Drivetrain").add("Left Pos", 0).getEntry();
        Robot.dashboardScheduler.addFunction(() -> sbLeftPos.setDouble(getLeftPosition()));
        sbRightPos = Shuffleboard.getTab("Drivetrain").add("Right Pos", 0).getEntry();
        Robot.dashboardScheduler.addFunction(() -> sbRightPos.setDouble(getRightPosition()));

        // commanded speeds
        sbLeftCommandedSpeed = Shuffleboard.getTab("Drivetrain").add("Left Cmmd Spd", 0).getEntry();
        sbRightCommandedSpeed = Shuffleboard.getTab("Drivetrain").add("Right cmmd Spd", 0).getEntry();

        // Test constant voltage command
        constantDriveTestCommand = new DriveConstantVoltage(0);
        Shuffleboard.getTab("TestDrive").add("Constant Drive", constantDriveTestCommand);
        Shuffleboard.getTab("TestDrive").add("Increasing Drive", new DriveIncrease());
        Shuffleboard.getTab("TestDrive").add("Increasing Drive Vel", new DriveIncreaseVel());
        sbTestDriveVoltage = Shuffleboard.getTab("TestDrive").add("Constant Voltage", 0.).getEntry();
        sbLeftPredictedCurrentDraw = Shuffleboard.getTab("TestDrive").add("Left Pred Current", 0).getEntry();
        Robot.dashboardScheduler.addFunction(() -> sbLeftPredictedCurrentDraw.setDouble(leftTransmission
                .getCurrentDraw(leftDrive.getMotorOutputVoltage(), leftDrive.getSelectedSensorVelocity())));
        sbRightPredictedCurrentDraw = Shuffleboard.getTab("TestDrive").add("Right Pred Current", 0).getEntry();
        Robot.dashboardScheduler.addFunction(() -> sbRightPredictedCurrentDraw.setDouble(rightTransmission
                .getCurrentDraw(rightDrive.getMotorOutputVoltage(), rightDrive.getSelectedSensorVelocity())));
        sbCurrentLimit = Shuffleboard.getTab("TestDrive").add("Current Limit", currentLimit).getEntry();

        // Velocity PID tuning
        sbVelKp = Shuffleboard.getTab("DriveVel").add("kP", RobotMap.DRIVE_VEL_LOW_KP).getEntry();
        sbVelKi = Shuffleboard.getTab("DriveVel").add("kI", RobotMap.DRIVE_VEL_LOW_KI).getEntry();
        sbVelKd = Shuffleboard.getTab("DriveVel").add("kD", RobotMap.DRIVE_VEL_LOW_KD).getEntry();
        sbVelIZone = Shuffleboard.getTab("DriveVel").add("iZone", RobotMap.DRIVE_VEL_LOW_IZONE).getEntry();
        sbVelIMax = Shuffleboard.getTab("DriveVel").add("iMax", RobotMap.DRIVE_VEL_LOW_IMAX).getEntry();

        // Motion Profiling
        sbMoProLeftFF = Shuffleboard.getTab("DrivePro").add("Left FF", 0).getEntry();
        Robot.dashboardScheduler.addFunction(() -> sbMoProLeftFF.setDouble(leftDrive.getActiveTrajectoryArbFeedFwd()));
        sbMoProRightFF = Shuffleboard.getTab("DrivePro").add("Right FF", 0).getEntry();
        Robot.dashboardScheduler
                .addFunction(() -> sbMoProRightFF.setDouble(rightDrive.getActiveTrajectoryArbFeedFwd()));
        sbMoProLeftPos = Shuffleboard.getTab("DrivePro").add("Left Pos", 0).getEntry();
        Robot.dashboardScheduler.addFunction(() -> sbMoProLeftPos.setDouble(leftDrive.getActiveTrajectoryPosition()));
        sbMoProRightPos = Shuffleboard.getTab("DrivePro").add("Right Pos", 0).getEntry();
        Robot.dashboardScheduler.addFunction(() -> sbMoProRightPos.setDouble(rightDrive.getActiveTrajectoryPosition()));
        sbMoProLeftVel = Shuffleboard.getTab("DrivePro").add("Left Vel", 0).getEntry();
        Robot.dashboardScheduler.addFunction(() -> sbMoProLeftVel.setDouble(leftDrive.getActiveTrajectoryVelocity()));
        sbMoProRightVel = Shuffleboard.getTab("DrivePro").add("Right Vel", 0).getEntry();
        Robot.dashboardScheduler.addFunction(() -> sbMoProRightVel.setDouble(rightDrive.getActiveTrajectoryVelocity()));
        sbMoProP = Shuffleboard.getTab("DrivePro").add("P", moProKP).getEntry();
        Robot.dashboardScheduler.addFunction(() -> {
            double newMoProP = sbMoProP.getDouble(driveConfiguration.left.masterConfiguration.slot0.kP);
            if (newMoProP != moProKP) {
                moProKP = newMoProP;
                leftDrive.config_kP(0, moProKP);
                rightDrive.config_kP(0, moProKP);
            }
        });
        sbMoProI = Shuffleboard.getTab("DrivePro").add("I", moProKI).getEntry();
        Robot.dashboardScheduler.addFunction(() -> {
            double newMoProI = sbMoProI.getDouble(driveConfiguration.left.masterConfiguration.slot0.kI);
            if (newMoProI != moProKI) {
                moProKI = newMoProI;
                leftDrive.config_kI(0, moProKI);
                rightDrive.config_kI(0, moProKI);
            }
        });
        sbMoProD = Shuffleboard.getTab("DrivePro").add("D", moProKD).getEntry();
        Robot.dashboardScheduler.addFunction(() -> {
            double newMoProD = sbMoProD.getDouble(driveConfiguration.left.masterConfiguration.slot0.kD);
            if (newMoProD != moProKD) {
                moProKD = newMoProD;
                leftDrive.config_kD(0, moProKD);
                rightDrive.config_kD(0, moProKD);
            }
        });

        SmartDashboard.putBoolean("ReverseTurnKyle", false);
    }

    public void updateShuffleboard() {
        // long startTime;

        // Talon status info
        // startTime = RobotController.getFPGATime();
        sbLeftDriveMode.setString(leftDrive.getControlMode().toString());
        sbRightDriveMode.setString(rightDrive.getControlMode().toString());
        // System.out.println("Talon status: " + (RobotController.getFPGATime() -
        // startTime));

        // Commanded Speeds
        // startTime = RobotController.getFPGATime();
        sbLeftCommandedSpeed.setDouble(leftCommandedSpeed);
        sbRightCommandedSpeed.setDouble(rightCommandedSpeed);
        // System.out.println("Commanded Speeds status: " +
        // (RobotController.getFPGATime()-startTime));

        // Test Constant Voltage Command
        // startTime = RobotController.getFPGATime();
        constantDriveTestCommand.setVoltage(sbTestDriveVoltage.getDouble(0));
        currentLimit = sbCurrentLimit.getDouble(RobotMap.DRIVE_CURRENT_LIMIT);
        // System.out.println("Voltage Command status: " +
        // (RobotController.getFPGATime()-startTime));

        // // Velocity PID tuning
        // startTime = RobotController.getFPGATime();
        if (resetFromShift) {
            sbVelKp.setDouble(leftVelocityController.getKP());
            sbVelKi.setDouble(leftVelocityController.getKI());
            sbVelKd.setDouble(leftVelocityController.getKD());
            sbVelIZone.setDouble(leftVelocityController.getIZone());
            sbVelIMax.setDouble(leftVelocityController.getIMax());
        } else {
            leftVelocityController.setKP(sbVelKp.getDouble(RobotMap.DRIVE_VEL_LOW_KP));
            leftVelocityController.setKI(sbVelKi.getDouble(RobotMap.DRIVE_VEL_LOW_KI));
            leftVelocityController.setKD(sbVelKd.getDouble(RobotMap.DRIVE_VEL_LOW_KD));
            leftVelocityController.setIZone(sbVelIZone.getDouble(RobotMap.DRIVE_VEL_LOW_IZONE));
            leftVelocityController.setIMax(sbVelIMax.getDouble(RobotMap.DRIVE_VEL_LOW_IMAX));
            rightVelocityController.setKP(sbVelKp.getDouble(RobotMap.DRIVE_VEL_LOW_KP));
            rightVelocityController.setKI(sbVelKi.getDouble(RobotMap.DRIVE_VEL_LOW_KI));
            rightVelocityController.setKD(sbVelKd.getDouble(RobotMap.DRIVE_VEL_LOW_KD));
            rightVelocityController.setIZone(sbVelIZone.getDouble(RobotMap.DRIVE_VEL_LOW_IZONE));
            rightVelocityController.setIMax(sbVelIMax.getDouble(RobotMap.DRIVE_VEL_LOW_IMAX));
        }
        // System.out.println("Velocity PID status: " +
        // (RobotController.getFPGATime()-startTime));

        resetFromShift = false;
    }

    public void basicDrive(double leftSpeed, double rightSpeed) {
        leftDrive.set(ControlMode.PercentOutput, leftSpeed);
        rightDrive.set(ControlMode.PercentOutput, rightSpeed);

    }

    /**
     * 
     * Commands the drivetrain to the given velocities (in fps) while proactively
     * limiting current draw.
     */
    public void basicDriveLimited(double leftVelocity, double rightVelocity) {
        leftCommandedSpeed = leftVelocity;
        rightCommandedSpeed = rightVelocity;
        leftDrive.setVelocityCurrentLimited(leftVelocity, currentLimit / 2);
        rightDrive.setVelocityCurrentLimited(rightVelocity, currentLimit / 2);
    }

    /**
     * Arcade drive function for teleop control.
     * 
     * Parameters:
     * 
     * @param speed    The forwards/backwards speed on a scale from -1 to 1
     * @param turnRate The rate to turn at on a scale from -1 (counterclockwise) to
     *                 1 (clockwise)
     */
    public void arcadeDrive(double speed, double turn) {
        speed *= maxSpeed;
        turn *= maxSpeed;
        joystickSpeed = speed;

        DriveUtils.signedPow(speed, RobotMap.DRIVE_SPEED_EXP);
        DriveUtils.signedPow(turn, RobotMap.DRIVE_TURN_EXP);

        turn = DriveUtils.cheesyTurn(speed, turn);

        if (SmartDashboard.getBoolean("ReverseTurnKyle", false) && speed < 0) {
            turn *= -1;
        }

        // speed = DriveUtils.signedPow(speed, 3);
        // turn = DriveUtils.signedPow(speed, 3);

        speed = limitAcceleration(speed);

        // turn = DriveUtils.cheesyTurn(speed, turn);

        double leftSpeed = (speed + turn);
        double rightSpeed = (speed - turn);

        basicDriveLimited(leftSpeed, rightSpeed);
        // basicDrive(speed + turn, speed - turn);
    }

    public double limitAcceleration(double speed) {
        double maxAccel;
        if (isInHighGear()) {
            if (Robot.m_arm.getDistanceFromBase() >= RobotMap.ARM_TIPPY_DISTANCE) {
                maxAccel = RobotMap.MAX_ACCEL_HIGH_TIPPY;
            } else {
                maxAccel = RobotMap.MAX_ACCEL_HIGH;
            }
        } else {
            if (Robot.m_arm.getDistanceFromBase() >= RobotMap.ARM_TIPPY_DISTANCE) {
                maxAccel = RobotMap.MAX_ACCEL_LOW_TIPPY;
            } else {
                maxAccel = RobotMap.MAX_ACCEL_LOW;
            }
        }

        if (speed - getStraightSpeed() > 0) {
            double vel = getStraightSpeed() + maxAccel;
            if (speed < vel) {
                return speed;
            } else {
                return vel;
            }
        } else {
            double vel = getStraightSpeed() - maxAccel;
            if (speed > vel) {
                return speed;
            } else {
                return vel;
            }

        }

    }

    /*
     * 
     * Old versions of arcade drive:
     * 
     * public void arcadeDrive(double speed, double turn) { double leftSpeed = speed
     * + turn; double rightSpeed = speed - turn; basicDrive(leftSpeed, rightSpeed);
     * 
     * }
     */

    public void resetVelocityPID() {
        leftVelocityController.reset();
        rightVelocityController.reset();
    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new ArcadeDrive());

    }

    public void setGearFromButton() {
        if (Robot.m_oi.getHighGearButton()) {
            this.shiftToHigh();
            maxSpeed = RobotMap.MAX_SPEED_HIGH;

        } else {
            this.shiftToLow();
            maxSpeed = RobotMap.MAX_SPEED_LOW;
        }
    }

    public void autoshift() {
        if (Robot.m_oi.getForceLowGearButton()) {
            this.shiftToLow();
            maxSpeed = RobotMap.MAX_SPEED_LOW;
        } else {
            if (isInHighGear() && Math.abs(getStraightSpeed()) <= RobotMap.SHIFT_INTO_LOW_GEAR) {
                shiftToLow();
            } else if (!isInHighGear() && Math.abs(getStraightSpeed()) >= RobotMap.SHIFT_INTO_HIGH_GEAR
                    && Math.abs(joystickSpeed) > RobotMap.COMMANDED_STOP_SPEED) {
                shiftToHigh();
            } else if (isInHighGear() && Math.abs(getStraightSpeed()) >= RobotMap.SHIFT_INTO_LOW_GEAR_STOP
                    && Math.abs(joystickSpeed) <= RobotMap.COMMANDED_STOP_SPEED) {
                shiftToLow();
            }
            maxSpeed = RobotMap.MAX_SPEED_HIGH;
        }
    }

    public void shiftToLow() {
        leftShifter.set(Value.kForward);
        rightShifter.set(Value.kForward);
        leftTransmission.shiftToLow();
        rightTransmission.shiftToLow();
        leftVelocityController.setKP(RobotMap.DRIVE_VEL_LOW_KP);
        leftVelocityController.setKI(RobotMap.DRIVE_VEL_LOW_KI);
        leftVelocityController.setKD(RobotMap.DRIVE_VEL_LOW_KD);
        leftVelocityController.setIZone(RobotMap.DRIVE_VEL_LOW_IZONE);
        leftVelocityController.setIMax(RobotMap.DRIVE_VEL_LOW_IMAX);
        rightVelocityController.setKP(RobotMap.DRIVE_VEL_LOW_KP);
        rightVelocityController.setKI(RobotMap.DRIVE_VEL_LOW_KI);
        rightVelocityController.setKD(RobotMap.DRIVE_VEL_LOW_KD);
        rightVelocityController.setIZone(RobotMap.DRIVE_VEL_LOW_IZONE);
        rightVelocityController.setIMax(RobotMap.DRIVE_VEL_LOW_IMAX);
        resetFromShift = true;
    }

    public void shiftToHigh() {
        leftShifter.set(Value.kReverse);
        rightShifter.set(Value.kReverse);
        leftTransmission.shiftToHigh();
        rightTransmission.shiftToHigh();
        leftVelocityController.setKP(RobotMap.DRIVE_VEL_HIGH_KP);
        leftVelocityController.setKI(RobotMap.DRIVE_VEL_HIGH_KI);
        leftVelocityController.setKD(RobotMap.DRIVE_VEL_HIGH_KD);
        leftVelocityController.setIZone(RobotMap.DRIVE_VEL_HIGH_IZONE);
        leftVelocityController.setIMax(RobotMap.DRIVE_VEL_HIGH_IMAX);
        rightVelocityController.setKP(RobotMap.DRIVE_VEL_HIGH_KP);
        rightVelocityController.setKI(RobotMap.DRIVE_VEL_HIGH_KI);
        rightVelocityController.setKD(RobotMap.DRIVE_VEL_HIGH_KD);
        rightVelocityController.setIZone(RobotMap.DRIVE_VEL_HIGH_IZONE);
        rightVelocityController.setIMax(RobotMap.DRIVE_VEL_HIGH_IMAX);
        resetFromShift = true;
    }

    public boolean isInHighGear() {
        return leftTransmission.isInHighGear();
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

    /**************************************************************************
     * MOTION PROFILING
     *************************************************************************/

    List<TJDriveMotionPoint> profile;
    BufferedTrajectoryPointStream m_leftTrajectoryBuffer = new BufferedTrajectoryPointStream();
    BufferedTrajectoryPointStream m_rightTrajectoryBuffer = new BufferedTrajectoryPointStream();

    public void loadMotionProfile(List<TJDriveMotionPoint> profile) {

        this.profile = profile;

        m_leftTrajectoryBuffer.Clear();
        m_rightTrajectoryBuffer.Clear();

        TrajectoryPoint leftPoint = new TrajectoryPoint();
        TrajectoryPoint rightPoint = new TrajectoryPoint();
        for (int i = 0; i < profile.size(); i++) {

            TJDriveMotionPoint tjPoint = profile.get(i);

            leftPoint.arbFeedFwd = leftTransmission.getFeedforwardVoltage(tjPoint.leftVelocity) / 12;
            rightPoint.arbFeedFwd = rightTransmission.getFeedforwardVoltage(tjPoint.rightVelocity) / 12;
            leftPoint.isLastPoint = (i == profile.size() - 1);
            rightPoint.isLastPoint = (i == profile.size() - 1);
            leftPoint.position = leftTransmission.convertOutputPositionToSensor(tjPoint.leftPosition);
            rightPoint.position = rightTransmission.convertOutputPositionToSensor(tjPoint.rightPosition);
            leftPoint.profileSlotSelect0 = 0;
            rightPoint.profileSlotSelect0 = 0;
            leftPoint.timeDur = (int) (tjPoint.dt * 1000);
            rightPoint.timeDur = (int) (tjPoint.dt * 1000);
            leftPoint.useAuxPID = false;
            rightPoint.useAuxPID = false;
            leftPoint.velocity = leftTransmission.convertOutputVelocityToSensor(tjPoint.leftVelocity);
            rightPoint.velocity = rightTransmission.convertOutputVelocityToSensor(tjPoint.rightVelocity);
            leftPoint.zeroPos = (i == 0);
            rightPoint.zeroPos = (i == 0);

            m_leftTrajectoryBuffer.Write(leftPoint);
            m_rightTrajectoryBuffer.Write(rightPoint);
        }
    }

    public void runMotionProfile() {
        leftDrive.startMotionProfile(m_leftTrajectoryBuffer, RobotMap.DRIVE_MIN_TRAJ_POINTS, ControlMode.MotionProfile);
        rightDrive.startMotionProfile(m_rightTrajectoryBuffer, RobotMap.DRIVE_MIN_TRAJ_POINTS,
                ControlMode.MotionProfile);

    }

    public boolean isProfileFinished() {
        return leftDrive.isMotionProfileFinished() && rightDrive.isMotionProfileFinished();
    }

}

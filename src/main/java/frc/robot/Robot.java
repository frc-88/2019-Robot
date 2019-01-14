/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.junit.Test.None;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.badlogs.BadLog;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import java.util.Date;
import java.text.SimpleDateFormat;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI m_oi;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private BadLog m_logger;
  private int loopsSinceLastLog = 0;
  private final int LOOPS_TO_WAIT_FOR_DISABLED_LOGGING = 50;

  public PowerDistributionPanel m_pdp;

  @Override
  public void robotInit() {

    // Initialize the PDP
    m_pdp = new PowerDistributionPanel(RobotMap.PDP_ID);

    // Initialize badlogs
    initializeLogging();



    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);



    // Finalize the BadLog initialization only after everything has had a
    // chance to register info
    m_logger.finishInitialization();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    updateLogs();

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * Initializes the BadLogs logging system, and adds some general logging
   * info to it.
   */
  private void initializeLogging() {

    // The file name
    String fileName = "/home/lvuser/log/";

    // Get the current timestamp
    String dateTime = new SimpleDateFormat("yyyy-MM-dd_HH:mm:ss").format(new Date());
    fileName += dateTime;

    // Check if we are in a match
    MatchType matchType = Optional.ofNullable(DriverStation.getInstance().getMatchType()).orElse(MatchType.None);
    String eventName = "";
    int matchNumber = 0;
    int replayNumber = 0;
    if (matchType != MatchType.None) {
      eventName = Optional.ofNullable(DriverStation.getInstance().getEventName()).orElse("");
      matchNumber = DriverStation.getInstance().getMatchNumber();
      replayNumber = DriverStation.getInstance().getReplayNumber();

      fileName += "_" + eventName + "_" + matchType.toString() + matchNumber + "(" + replayNumber + ")";
    }

    // Initialize the logger
    fileName += ".badlog";
    m_logger = BadLog.init(fileName);

    // Log session info
    BadLog.createValue("Timestamp", dateTime);
    BadLog.createValue("Is FMS Connected?", String.valueOf(DriverStation.getInstance().isFMSAttached()));
    BadLog.createValue("Event Name", eventName);
    BadLog.createValue("Match Type", matchType.toString());
    BadLog.createValue("Match Number", String.valueOf(matchNumber));
    BadLog.createValue("Replay Number", String.valueOf(replayNumber));
    BadLog.createValue("Alliance", Optional.ofNullable(DriverStation.getInstance().getAlliance()).orElse(Alliance.Invalid).toString());
    BadLog.createValue("DS Location", String.valueOf(DriverStation.getInstance().getLocation()));

    // Log the current time from start
    final long startTime = RobotController.getFPGATime(); // us
    BadLog.createTopic("Time", "s", () -> (RobotController.getFPGATime() - startTime) / 1e6, "hide", "delta", "xaxis");

    // Logging the current mode and match time
    BadLog.createTopic("Match time", "s", () -> DriverStation.getInstance().getMatchTime());
    BadLog.createTopicStr("Operation Mode", BadLog.UNITLESS, new Supplier<String>() {

      @Override
      public String get() {
        if (DriverStation.getInstance().isAutonomous()) {
          return "Auton";
        } else if (DriverStation.getInstance().isOperatorControl()) {
          return "Teleop";
        } else if (DriverStation.getInstance().isTest()) {
          return "Test";
        } else {
          return "Disabled";
        }
      }
    });

    // Log power draw information
    BadLog.createTopicStr("Is Browned Out?", BadLog.UNITLESS, () -> String.valueOf(RobotController.isBrownedOut()));
    BadLog.createTopic("Battery Voltage", "V", () -> m_pdp.getVoltage());
    BadLog.createTopic("Total Current Draw", "A", () -> m_pdp.getTotalCurrent());

    // Log CAN information
    BadLog.createTopic("CAN Bus Off Count", BadLog.UNITLESS, () -> (double)RobotController.getCANStatus().busOffCount);
    BadLog.createTopic("CAN Bus Utilization", "%", () -> RobotController.getCANStatus().percentBusUtilization);
    BadLog.createTopic("CAN RX Error Count", BadLog.UNITLESS, () -> (double)RobotController.getCANStatus().receiveErrorCount);
    BadLog.createTopic("CAN TX Error Count", BadLog.UNITLESS, () -> (double)RobotController.getCANStatus().transmitErrorCount);
    BadLog.createTopic("CAN TX Full Count", BadLog.UNITLESS, () -> (double)RobotController.getCANStatus().txFullCount);

  }

  /**
   * Log all topics for the Bad Logs logging system, if it is time to
   */
  private void updateLogs() {

    m_logger.updateTopics();
    if (DriverStation.getInstance().isEnabled() 
        || loopsSinceLastLog > LOOPS_TO_WAIT_FOR_DISABLED_LOGGING) {
      
      m_logger.log();
      loopsSinceLastLog = 0;

    } else {
      loopsSinceLastLog++;
    }

  }

}

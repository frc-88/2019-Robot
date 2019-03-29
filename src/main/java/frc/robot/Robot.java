/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SAPG;
import frc.robot.util.TimeScheduler;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static Arm m_arm;
  public static SAPG m_sapg;
  public static Climber m_climber;
  public static Limelight m_limelight_sapg;
  public static NavX m_navx;
  public static Drive m_drive;
  public static OI m_oi;
  public static Intake m_intake;
  public static Compressor compressor;

  public static NetworkTableEntry soundPlaying;

  public static TimeScheduler dashboardScheduler;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  long lastTeleopPerStart = Long.MAX_VALUE;
  long lastTeleopPerEnd = Long.MAX_VALUE;
  long lastRobotPerEnd = Long.MAX_VALUE;
  long lastControlPacket = Long.MAX_VALUE;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    CameraServer.getInstance().startAutomaticCapture();

    compressor = new Compressor(RobotMap.COMPRESSOR_PCM);
    m_navx = new NavX();
    m_limelight_sapg = new Limelight("limelight-sapg");
    m_drive = new Drive();
    m_intake = new Intake();
    m_arm = new Arm();
    m_sapg = new SAPG();
    m_climber = new Climber();
    

    // instantiate m_oi last...it may reference subsystems
    m_oi = new OI();

    initializeDashboard();

    m_limelight_sapg.ledOff();
    m_limelight_sapg.camDriver();

    m_navx.zeroPitch();

    // m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);

    // NetworkTableInstance.getDefault().
    soundPlaying = NetworkTableInstance.getDefault().getTable("alerts").getEntry("sound");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    if (RobotMap.DEBUG) writeDashboard();

    makeSounds();

    lastRobotPerEnd = RobotController.getFPGATime();
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
    m_arm.zero();
    m_arm.configureCoastMode();

    soundPlaying.setString("");
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();

    m_limelight_sapg.ledOff();
    m_limelight_sapg.camDriver();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_arm.zero();
    m_arm.configureBrakeMode();

    m_limelight_sapg.ledOff();
    m_limelight_sapg.camDriver();

    m_navx.zeroPitch();

    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
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
    m_arm.zero();
    m_arm.configureBrakeMode();

    m_limelight_sapg.ledOff();
    m_limelight_sapg.camDriver();

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

    long timeNow = RobotController.getFPGATime();
    long timeTeleopPer = lastTeleopPerEnd - lastTeleopPerStart;
    long timeRobotPer = lastRobotPerEnd - lastTeleopPerEnd;
    long timeOther = timeNow - lastRobotPerEnd;
    long timePacket = timeNow - lastControlPacket;

    System.out.println("TIMING - TeleopPer: " + timeTeleopPer
        + "   RobotPer: " + timeRobotPer
        + "   Other: " + timeOther
        + "   LastPacket: " + timePacket);

    if (timeTeleopPer > 250_000) {
      System.out.println("WARNING LOOK AT ME: TELEOP TOOK A LONG TIME");
    }
    if (timeRobotPer > 250_000) {
      System.out.println("WARNING LOOK AT ME: ROBOT TOOK A LONG TIME");
    }
    if (timeOther > 250_000) {
      System.out.println("WARNING LOOK AT ME: OTHER TOOK A LONG TIME");
    }
    if (timePacket > 250_000) {
      System.out.println("WARNING LOOK AT ME: PACKET TOOK A LONG TIME");
    }

    lastTeleopPerStart = timeNow;
    if (DriverStation.getInstance().isNewControlData()) {
      lastControlPacket = timeNow;
    }

    Scheduler.getInstance().run();

    lastTeleopPerEnd = RobotController.getFPGATime();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  boolean surprise = false;
  long lastLoopTime = Long.MAX_VALUE;
  int hasTargetCounts = 0;
  int noTargetCounts = 0;
  int hasPanelCounts = 0;
  int noPanelCounts = 0;
  int hasCargoCounts = 0;
  int noCargoCounts = 0;
  int lastNumPlayed = -1;

  public void makeSounds() {

    if (m_limelight_sapg.hasTarget()) {
      hasTargetCounts++;
    } else {
      noTargetCounts++;
    }
    if (m_intake.hasCargo()) {
      hasCargoCounts++;
    } else {
      noCargoCounts++;
    }
    if (m_sapg.hasPanel()) {
      hasPanelCounts++;
    } else {
      noPanelCounts++;
    }

    // if (m_sapg.isTracking() && hasTargetCounts == 5) {

    //   // Target Acquired
    //   soundPlaying.setString("i_see_you");
    //   noTargetCounts = 0;

    // }

    // if (m_sapg.isTracking() && noTargetCounts == 5) {

    //   // Target Lost
    //   soundPlaying.setString("cant_see_me");
    //   hasTargetCounts = 0;

    // }

    if (hasCargoCounts == 5) {

      // Got cargo
      soundPlaying.setString("cargo");
      noCargoCounts = 0;

    }

    if (noCargoCounts == 5) {

      // Lost cargo
      soundPlaying.setString("oopsie_daisy");
      hasCargoCounts = 0;

    }

    if (hasPanelCounts == 5) {

      // Got cargo
      soundPlaying.setString("got_it");
      noCargoCounts = 0;

    }

    if (noPanelCounts == 5) {

      // Lost cargo
      soundPlaying.setString("oopsie_daisy");
      hasCargoCounts = 0;

    }

    if (RobotController.isBrownedOut()) {

      // Brownout
      soundPlaying.setString("power");

    }

    if (RobotController.getFPGATime() - lastLoopTime > 1 * 1_000_000) {

      // Comms blip
      soundPlaying.setString("comms");

    }

    if (!surprise && DriverStation.getInstance().isFMSAttached() && DriverStation.getInstance().isAutonomous() && Math.abs(m_drive.getStraightSpeed()) > 2) {

      // Shhhh! This is a surprise for Brad. Keep it a secret.
      surprise = true;
      soundPlaying.setString("surprise");

    }

    lastLoopTime = RobotController.getFPGATime();
  }

  private void initializeDashboard() {
    dashboardScheduler = new TimeScheduler();
    m_drive.configureShuffleboard();
  }

  private void writeDashboard() {
    final long RUN_TIME = 2;
    dashboardScheduler.run(RUN_TIME);

    m_drive.updateShuffleboard();
    m_arm.updateDashboard();
    m_sapg.updateDashboard();
    m_intake.updateDashboard();
    m_climber.updateDashboard();
    m_navx.updateDashboard();
    m_limelight_sapg.updateDashboard();

    SmartDashboard.putNumber("Match Time", DriverStation.getInstance().getMatchTime());
  }
}

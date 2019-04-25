/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DrivePlayback extends Command {
  private List<List<Double>> points = new ArrayList<>();
  private String pathFile;
  private double startTime;
  private int idx;

  public DrivePlayback() {
    requires(Robot.m_drive);
    Preferences prefs = Preferences.getInstance();
    pathFile = "/home/lvuser/" + prefs.getString("Drive:RecordFile", "test");
  }

  public DrivePlayback(String filename) {
    requires(Robot.m_drive);

    pathFile = filename;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    try {
      BufferedReader br = new BufferedReader(new FileReader(new File(pathFile)));

      String line;
      double initTime = -1;
      while ((line = br.readLine()) != null) {
        String[] values = line.split(",");
        if (values.length > 0) {
          List<Double> converted = new ArrayList<Double>();
          if (initTime == -1) {
            initTime = Long.parseLong(values[0]);
            converted.add(0.);
          } else {
            converted.add(Double.parseDouble(values[0]) - initTime);
          }
          converted.add(Double.parseDouble(values[1]));
          converted.add(Double.parseDouble(values[2]));
          points.add(converted);
        }
      }

      br.close();
    } catch (IOException ioe) {
      ioe.printStackTrace();
    }

    startTime = RobotController.getFPGATime();
    idx = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double timeElapsed = RobotController.getFPGATime() - startTime;
    while (idx < points.size()) {
      System.out.println(timeElapsed + "  " + points.get(idx).get(0));
      if (timeElapsed > points.get(idx).get(0) + 10000) {
        idx++;
      } else {
        break;
      }
    }

    Robot.m_drive.autoshift();

    double speed = points.get(idx).get(1);
    double turn = points.get(idx).get(2);

    Robot.m_drive.arcadeDrive(speed, turn);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return idx == points.size() - 1;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

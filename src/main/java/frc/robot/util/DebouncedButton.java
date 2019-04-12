package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.buttons.Trigger;

public class DebouncedButton extends Trigger {

    boolean counting = false;
    private long startTime;

    Trigger m_button;

    public DebouncedButton(Trigger button) {
        m_button = button;
    }

    @Override
    public boolean get() {
        if (m_button.get()) {
            if (counting) {
              return RobotController.getFPGATime() - startTime > 250000;
            } else {
              startTime = RobotController.getFPGATime();
              counting = true;
              return false;
            }
        } else {
        counting = false;
        return false;
        }
        
    }

}
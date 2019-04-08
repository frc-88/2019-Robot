package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class TJButtonBox extends Joystick {
	private static final int BUTTON_RED_LOW = 1;
	private static final int BUTTON_RED_CARGO = 2;
	private static final int BUTTON_RED_MID = 3;
	private static final int BUTTON_RED_HIGH = 4;
	private static final int BUTTON_BLUE_LOW = 5;
	private static final int BUTTON_BLUE_CARGO = 6;
	private static final int BUTTON_BLUE_MID = 7;
	private static final int BUTTON_BLUE_HIGH = 8;
	private static final int BUTTON_WHITE_HOME = 9;
	private static final int BUTTON_RED_BIG = 10;
	private static final int BUTTON_WHITE_TRIANGLE = 11;
	private static final int BUTTON_YELLOW_TRIANGLE = 12;
	private static final int BUTTON_WHITE_LEFT_TOP = 13;
	private static final int BUTTON_WHITE_TOP_LEFT = 14;
	private static final int BUTTON_GREEN_TRIANGLE = 15;
	private static final int BUTTON_WHITE_TOP_RIGHT = 16;
	private static final int BUTTON_WHITE_RIGHT_TOP = 17;
	private static final int BUTTON_BBB_RED = 18;
	private static final int BUTTON_BBB_BLUE = 19;
	private static final int BUTTON_BBB_SWITCH = 20;
	
	public Button buttonRedLow = new JoystickButton(this, BUTTON_RED_LOW);
	public Button buttonRedCargo = new JoystickButton(this, BUTTON_RED_CARGO);
	public Button buttonRedMid = new JoystickButton(this, BUTTON_RED_MID);
	public Button buttonRedHigh = new JoystickButton(this, BUTTON_RED_HIGH);
	public Button buttonBlueLow = new JoystickButton(this, BUTTON_BLUE_LOW);
	public Button buttonBlueCargo = new JoystickButton(this, BUTTON_BLUE_CARGO);
	public Button buttonBlueMid = new JoystickButton(this, BUTTON_BLUE_MID);
	public Button buttonBlueHigh = new JoystickButton(this, BUTTON_BLUE_HIGH);
	public Button buttonWhiteHome = new JoystickButton(this, BUTTON_WHITE_HOME);
	public Button buttonRedBig = new JoystickButton(this, BUTTON_RED_BIG);
	public Button buttonWhiteTriangle = new JoystickButton(this, BUTTON_WHITE_TRIANGLE);
	public Button buttonYellowTriangle = new JoystickButton(this, BUTTON_YELLOW_TRIANGLE);
	public Button buttonWhiteLeftTop = new JoystickButton(this, BUTTON_WHITE_LEFT_TOP);
	public Button buttonWhiteTopLeft = new JoystickButton(this, BUTTON_WHITE_TOP_LEFT);
	public Button buttonGreenTriangle = new JoystickButton(this, BUTTON_GREEN_TRIANGLE);
	public Button buttonWhiteTopRight = new JoystickButton(this, BUTTON_WHITE_TOP_RIGHT);
	public Button buttonWhiteRightTop = new JoystickButton(this, BUTTON_WHITE_RIGHT_TOP);
	public Button buttonBBBRed = new JoystickButton(this, BUTTON_BBB_RED);
	public Button buttonBBBBlue = new JoystickButton(this, BUTTON_BBB_BLUE);
	public Button buttonBBBSwitch = new JoystickButton(this, BUTTON_BBB_SWITCH);

	public TJButtonBox(int port) {
		super(port);
	}
}
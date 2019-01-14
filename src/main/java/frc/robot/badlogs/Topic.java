package frc.robot.badlogs;

abstract class Topic extends NamespaceObject {
	public abstract String getUnit();
	public abstract String[] getAttributes();
	public abstract String getValue();
}

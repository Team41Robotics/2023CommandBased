package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Util;
import frc.robot.subsystems.HDriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
public class Drive extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    Joystick leftJoy,rightJoy;
    HDriveSubsystem drive = HDriveSubsystem.getInstance();

    public Drive(){
        addRequirements(drive);
        leftJoy = Robot.leftJoy;
        rightJoy = Robot.rightJoy;
    }
    @Override
    public void execute() {
        double vf = -Util.deadZone(leftJoy.getY());
        double vs = -Util.deadZone(leftJoy.getX());
        double omega = -Util.deadZone(rightJoy.getX());
        drive.drive(vf, vs, omega);
    }
}

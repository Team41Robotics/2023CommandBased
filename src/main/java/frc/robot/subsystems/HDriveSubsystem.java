package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX
import com.ctre.phoenix.motorcontrol.ControlMode;


public class HDriveSubsystem extends SubsystemBase {
	public ShuffleboardTab dttab = Shuffleboard.getTab("Drivetrain");
	public Encoder lef_enc = new Encoder(0, 1, true);
	public Encoder mid_enc = new Encoder(2, 3);
	public Encoder rgt_enc = new Encoder(4, 5);

	static HDriveSubsystem hdrive;

    TalonFX bot_lef = new TalonFX(DrivetrainConstants.BOTTOM_LEFT);
    TalonFX top_lef = new TalonFX(DrivetrainConstants.TOP_LEFT);
    TalonFX mid = new TalonFX(DrivetrainConstants.MID);
    TalonFX top_rgt = new TalonFX(DrivetrainConstants.TOP_RIGHT);
    TalonFX bot_rgt = new TalonFX(DrivetrainConstants.BOTTOM_RIGHT);
    
    /* 
     public Encoder lef_enc = new Encoder(0, 1, true);
     public Encoder mid_enc = new Encoder(2, 3);
     public Encoder rgt_enc = new Encoder(4, 5);
    */

	double vl, vr, vm;
	double vx, vy, w;

    public HDriveSubsystem() {
        super();

        /* 
        lef_enc.setDistancePerPulse(0.05 * 2 * Math.PI / 90);
        mid_enc.setDistancePerPulse(0.05 * 2 * Math.PI / 90);
        rgt_enc.setDistancePerPulse(0.05 * 2 * Math.PI / 90);

        dttab.add(this);
        dttab.addNumber("le", () -> lef_enc.getDistance());
        dttab.addNumber("me", () -> mid_enc.getDistance());
        dttab.addNumber("re", () -> rgt_enc.getDistance());
        */
        
        dttab.addNumber("vl", () -> vl);
        dttab.addNumber("vr", () -> vr);
        dttab.addNumber("vm", () -> vm);

		dttab.addNumber("vx", () -> vx);
		dttab.addNumber("vy", () -> vy);
		dttab.addNumber("w", () -> w);
	}

	public void drive(double vx, double vy, double w) {
		drive(vx, vy, w, true);
	}

	public void drive(double vx, double vy, double w, boolean preserve) {
		this.vx = vx;
		this.vy = vy;
		this.w = w;
		vl = -vx + w;
		vr = -vx - w;
		vm = vy;

		double max = Math.max(Math.max(Math.abs(vl), Math.abs(vm)), Math.abs(vr));
		if (max > 1) {
			vx /= max;
			vy /= max;
			w /= max;
			vl = -vx + w;
			vr = -vx - w;
			vm = vy;
		}

        //vl/=2; vr/=2; vm/=2; // avoid brownout
        //vl=MathUtil.clamp(vl, -.5, .5);
        //vr=MathUtil.clamp(vr, -.5, .5);
        //vm=MathUtil.clamp(vm, -.5, .5);
        bot_lef.set(ControlMode.Velocity, vl);
        top_lef.set(ControlMode.Velocity, vl);
        mid.set(ControlMode.Velocity, vm);
        top_rgt.set(ControlMode.Velocity, vr);
        bot_rgt.set(ControlMode.Velocity, vr);
    }

	public static HDriveSubsystem getInstance() {
		if (hdrive == null) {
			hdrive = new HDriveSubsystem();
		}
		return hdrive;
	}
}

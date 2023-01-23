package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class PIDController implements Sendable {
        private double kP, kI, kD, I_thres;

        public PIDController(double kP, double kI, double kD, double I_thres) {
                this.kP = kP;
                this.kI = kI;
                this.kD = kD;
                this.I_thres = I_thres;
        }
        public double getkP() { return kP; }
        public double getkI() { return kI; }
        public double getkD() { return kD; }
        public void setkP(double kP) { this.kP = kP; }
        public void setkI(double kI) { this.kI = kI; }
        public void setkD(double kD) { this.kD = kD; }
        public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("PIDController");
                builder.addDoubleProperty("p", this::getkP, this::setkP);
                builder.addDoubleProperty("i", this::getkI, this::setkI);
                builder.addDoubleProperty("d", this::getkD, this::setkD);
        }

        double perror=0;
        double pt=0;
        double integral=0;
        public double calculate(double error) {
                double time = Timer.getFPGATimestamp();
                double dt = time - pt;
                if(dt > 0.1) {
                        pt = time; return 0;
                }
                double derivative = (error-perror)/dt;
                if (Math.abs(error) > I_thres)
                        integral = 0;
                integral += error * dt;

                pt = time;
                perror = error;
                return kP * error + kI * integral + kD * derivative;
        }
}

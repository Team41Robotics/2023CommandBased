package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class PIDController {
        double kP, kI, kD;
        double I_thres;

        public PIDController(double kP, double kI, double kD, double I_thres) {
                this.kP = kP;
                this.kI = kI;
                this.kD = kD;
                this.I_thres = I_thres;
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

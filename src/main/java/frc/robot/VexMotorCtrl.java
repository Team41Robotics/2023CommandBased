package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

public class VexMotorCtrl extends PWMMotorController {

        public VexMotorCtrl(String name, int channel) {
                super(name, channel);
        }
}
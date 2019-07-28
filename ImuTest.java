package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "IMU: Z Axis", group = "TESTING")
public class ImuTest extends Autonom {

    @Override
    public void runOpMode() {
        
        super.init(hardwareMap);
        
        waitForStart();
        
        while (!isStopRequested()) {
            super.rotate(2240);
            super.setPower(0.1f);
            while (super.heading > -90) {
                super.readAngle();
                telemetry.addData("Z", super.heading);
                telemetry.update();
            }
            super.stop();
        }
    }
}
    

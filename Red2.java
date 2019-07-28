package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name="RED2", group="AUTONOMOUS")
public class Red2 extends Autonom {

    private int leftUnits = 1120 *  107 / 100;
    private int centerUnits = 1120 * 68 / 100;
    private int rightUnits = 1120 * 16 / 100;
    private int vuMarkUnits = 0;

    @Override
    public void runOpMode() {

        super.init(hardwareMap);

        waitForStart();

        super.relicTrackables.activate();

        while (opModeIsActive()) {
            //color
            if (!super.task1)
            {
               super.scoreJewel(1);
               super.telemetryHandler();
            }
            //vuMark
            if(super.task2 == false) {
                for(int i = 0; i < 40; i++) {
                    super.vuforiaScan();
                    super.telemetryHandler();
                    if (super.vuMark != RelicRecoveryVuMark.UNKNOWN) {break;}
                }
                switch (super.vuMark) {
                    case LEFT:
                        vuMarkUnits = leftUnits;
                        break;
                    case RIGHT:
                        vuMarkUnits = rightUnits;
                        break;
                    default:
                        vuMarkUnits = centerUnits;
                        break;
                }
                super.task2 = true;
            }
            if (super.task3 == false) {

                super.moveForward(1120 * 180 / 100);
                super.waitToFinish();

                super.rotate(1120 * 25 / 10);
                super.waitToFinish();

                super.moveRight(vuMarkUnits);
                super.waitToFinish();



                super.qubeServos(true);
                sleep(750);

                super.moveForward(1120 / 5);
                super.waitToFinish();

                super.qubeServos(false);

                // super.moveForward(1120);
                // super.waitToFinish();
                //
                // super.rotate(1120 * 13 / 10); 90 de grade
                // super.waitToFinish();

                super.task3 = true;
            }
            super.telemetryHandler();
            super.stop();
        }
    }
}

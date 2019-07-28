/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.RobotHw;
import com.qualcomm.hardware.bosch.BNO055IMU;


public class Autonom extends LinearOpMode {

    // essentials
    RobotHw robot = new RobotHw();
    public float heading = 0;
    public int left_front_encoder;
    public int right_front_encoder;
    public int left_back_encoder;
    public int right_back_encoder;
    public static final int UNIT = 1120;   // ticks in a full wheel rotation
    public boolean task1 = false;  //  jewel scoring
    public boolean task2 = false;  //  vuforia scanning
    public boolean task3 = false;  //  scoring the glyph in the cryptobox
    public RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
    public String jewelColour = "unknown";


    // VUFORIA, DO NOT TOUCH
    VuforiaLocalizer vuforia;
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    // ------------------------------------------------------------------------------------------------

    public void init(HardwareMap hwMap) {
        robot.init(hwMap);
        robot.useEncoder(true);
        colorServoState(false);
        robot.colorSensor.enableLed(true);
        robot.setFloat(false);
        parameters.vuforiaLicenseKey = "AX9Tt1f/////AAAAmZgP0a7xc00OtCKro8AxNIB0Ga4OEzIbXne8gRzbrIWLbdDHycjj1xMWsNBKrsRDAEiWKN3LTwH77nbmYSVM4LeDtJodEejVAlG6J+U0m1UhTz8+tvYYfOSptuQWhhIo3bJQS/ZZo0IO/emnTgjgcBU9S9dMdIxFTHA71SPKsoMaNyE+yFGgVur3UbU9uY0oPBzRnPEFW4GB1hVaV8LQHLRYCd7/6H1fqDggix7GCKpNzw5y/1mCTGJxTRTI+j//ZhvtW6PUCHYxPyRLUzjy/wTettLuB6oTslQDW3UTuxq6dCjOHJskVNVWVkFlTiWk1oxkjqhBd6Px60nwHHNL038o2WZUitk2whtkFkhB6DYX";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        telemetryHandler();
    }
    // -------------------------------------------------------------------------

    // jewel
    public int readColor() {return robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER);}

    public void colorServoState(boolean bool) {
        // if true, the robotic arms unfolds and goes down
        if (bool) {
            robot.colorServo.setPosition(0.55f);
            sleep(300);
            robot.colorServo2.setPosition(0.15f);
            sleep(300);
            sleep(300);
            robot.colorServo.setPosition(0.8f);
            return;
        }
        // otherwise, it starts folding and goes up
        robot.colorServo.setPosition(0.55f);
        sleep(300);
        robot.colorServo2.setPosition(1f);
        sleep(300);
        robot.colorServo.setPosition(0.17f);
        sleep(300);
        return;
    }



    public void scoreJewel(int alliance) {
        // the robotic arm goes down
        telemetryHandler();
        colorServoState(true);
        sleep(750);
        int color = 0;
        // begin scanning the jewel
        for (int i = 0; i < 50; i++) {
            color = readColor();
            telemetryHandler();
            if (color != 0) {break;}
            sleep(10);
            idle();
        }
        // 1=red, 2=blue
        // score the jewel according to the jewel scanned and your alliance
        if (alliance == 1) {
            if (color == 3) {
                robot.colorServo2.setPosition(0f);
            }
            else if (color == 10) {
                robot.colorServo2.setPosition(0.4f);
            }
            else {
                robot.colorServo2.setPosition(0.17f);
            }
        }
        else {
            if (color == 3) {
                robot.colorServo2.setPosition(0.4f);
            }
            else if (color == 10) {
                robot.colorServo2.setPosition(0f);
            }
            else {
                robot.colorServo2.setPosition(0.17f);
            }
        }
        sleep(250);
        // lift the robotic arm
        colorServoState(false);
        if (color == 3) { jewelColour = "Blue"; }
        if (color == 10) { jewelColour = "Red"; }
        task1 = true;
        telemetryHandler();
    }
    // -------------------------------------------------------------------------

    // movement
    // function to refresh the encoder reading variables
    public void refresh() {
        left_front_encoder = robot.left_front.getCurrentPosition();
        left_back_encoder = robot.left_back.getCurrentPosition();
        right_front_encoder = robot.right_front.getCurrentPosition();
        right_back_encoder = robot.right_back.getCurrentPosition();
    }

    // function to set a target(a number of encoder ticks that a motor has to get to)
    // the values are given according to a clockwise arrangement of the motors
    // the first motor is the front motor on the left side
    public void setTarget(int a, int b, int c, int d) {
        robot.left_front.setTargetPosition(a);
        robot.right_front.setTargetPosition(b);
        robot.right_back.setTargetPosition(c);
        robot.left_back.setTargetPosition(d);
    }

    // function to set a power to all the drive train motors
    public void setPower(float a) {
        robot.left_front.setPower(a);
        robot.right_front.setPower(a);
        robot.right_back.setPower(a);
        robot.left_back.setPower(a);
    }

    // function that moves the robot forward a number of units
    // this works by calculating the encoder tick every motor has to get to
    // this also sets a 0.35f power to all drive train motors
    // in order to move slower or faster, after this function use the
    // setPower(float) function
    public void moveForward(int units) {
        refresh();
        setPower(0.35f);
        setTarget(left_front_encoder - units, right_front_encoder + units, right_back_encoder + units, left_back_encoder - units);
    }

    // function that moves the robot backward a number of units
    // see moveForward(int) for more details aboout how this works
    public void moveBackward(int units) {
        refresh();
        setPower(0.35f);
        setTarget(left_front_encoder + units, right_front_encoder - units, right_back_encoder - units, left_back_encoder + units);
    }

    // function that moves the robot left a number of units
    // see moveForward(int) for more details aboout how this works
    public void moveLeft(int units) {
        refresh();
        setPower(0.35f);
        setTarget(left_front_encoder + units, right_front_encoder + units, right_back_encoder - units, left_back_encoder - units);
    }

    // function that moves the robot right a number of units
    // see moveForward(int) for more details aboout how this works
    public void moveRight(int units) {
        refresh();
        setPower(0.35f);
        setTarget(left_front_encoder - units, right_front_encoder - units, right_back_encoder + units, left_back_encoder + units);
    }

    // function that rotates the robot a number of units
    // positive units rotates the robot right
    // negative units rotates the robot left
    // this works by giving each motor the same encoder tick it has to get to
    public void rotate(int units) {
        refresh();
        setPower(0.35f);
        setTarget(left_front_encoder - units, right_front_encoder - units, right_back_encoder - units, left_back_encoder - units);
    }

    // fucntion that stops the robot
    public void stopPower() {
        setPower(0f);
    }

    public void waitToFinish() {
        while ( robot.left_front.isBusy() && robot.right_front.isBusy() && robot.left_back.isBusy() && robot.right_back.isBusy() ) { telemetryHandler(); }
        sleep(650);
    }
    // -------------------------------------------------------------------------

    // others
    public void qubeServos(boolean ramp){
        if (ramp == true){
            robot.ramp1.setPosition(0.13);
            robot.ramp2.setPosition(0.13);
        }
        else{
            robot.ramp1.setPosition(0.97);
            robot.ramp2.setPosition(0.97);
        }
    }

    @Override
    public void runOpMode() {   }

    public void vuforiaScan()
    {
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        telemetryHandler();
    }
    // -------------------------------------------------------------------------

    public void telemetryHandler() {
        telemetry.addData("Jewel task completion", task1);
        telemetry.addData("Vuforia task completion", task2);
        telemetry.addData("Glyph task completion", task3);
        telemetry.addData("Scanned VuMark", vuMark);
        telemetry.addData("Detected colour", jewelColour);
        telemetry.update();
    }
    
    public void readAngle() {
        heading = robot.leftIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}

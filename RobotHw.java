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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;

public class RobotHw {
    // hardware map for configuring all hardware device
    public HardwareMap hardwareMap =  null;

    // drive train motors
    public DcMotor left_back = null;
    public DcMotor left_front = null;
    public DcMotor right_front = null;
    public DcMotor right_back = null;

    // other motors
    public DcMotor left_qubes = null;
    public DcMotor right_qubes = null;
    public DcMotor lift = null;
    public DcMotor relic = null;

    // sensors
    public ModernRoboticsI2cColorSensor colorSensor = null;
    public BNO055IMU leftIMU = null;
    public BNO055IMU rightIMU = null;

    // servos
    public Servo colorServo = null;
    public Servo ramp1 = null;
    public Servo ramp2 = null;
    public Servo colorServo2 = null;
    public CRServo relicServo = null;
    public Servo relicClaw = null;

    // constructor, don't need it (IGNORE THIS)
    public RobotHw(){

    }

    // initialize the hardware devices using the hardware map of the robot(give this function the hardwareMap
    // used in an opmode to configure hardware devices)
    public void init(HardwareMap hwMap) {
        // hardware map aquired from the op mode
        hardwareMap = hwMap;
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        // parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        // parameters.loggingEnabled      = true;
        // parameters.loggingTag          = "IMU";
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // drive train motors
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");

        // other motors
        left_qubes = hardwareMap.get(DcMotor.class, "left_qubes");
        right_qubes = hardwareMap.get(DcMotor.class, "right_qubes");
        lift = hardwareMap.get(DcMotor.class, "lift");
        relic = hardwareMap.get(DcMotor.class, "relic");

        // servos
        ramp1 = hardwareMap.get(Servo.class, "ramp1");
        ramp2 = hardwareMap.get(Servo.class, "ramp2");
        colorServo = hardwareMap.get(Servo.class, "colorServo");
        colorServo2 = hardwareMap.get(Servo.class, "colorServo2");
        relicServo = hardwareMap.get(CRServo.class, "relicServo");
        relicClaw = hardwareMap.get(Servo.class, "relicClaw");
        
        // sensors
        colorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "colorSensor");
        leftIMU = hardwareMap.get(BNO055IMU.class, "leftIMU");
        rightIMU = hardwareMap.get(BNO055IMU.class, "rightIMU");
        leftIMU.initialize(parameters);
        rightIMU.initialize(parameters);

        // zero power behaviour
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_qubes.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_qubes.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // relic.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // configure the drive train to work in the driver period
    public void configDriverPeriod() {
        left_front.setDirection(DcMotor.Direction.FORWARD);
        right_front.setDirection(DcMotor.Direction.FORWARD);
        left_back.setDirection(DcMotor.Direction.FORWARD);
        right_back.setDirection(DcMotor.Direction.FORWARD);
    }

    public void setFloat(boolean bool) {
        if (bool) {
            left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            right_qubes.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            left_qubes.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            return;
        }
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_qubes.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_qubes.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return;
    }


    // // method for colorServo
    // public void colorServoPos(boolean color) {
        // if (color) {
            // // robot.colorServo.setPosition(0.28f);
            // for (float i = 0.89f; i>=0.28f; i-=0.01f) {
                // colorServo.setPosition(i);
                // // sleep(1);
            // }
        // }
        // else {
            // // robot.colorServo.setPosition(0.89f);
            // for (float i = 0.28f; i<=0.89f; i+=0.01f) {
                // colorServo.setPosition(i);
                // // sleep(1);
            // }
        // }
        // // sleep(50);
    // }

    // public void colorServoState(boolean bool) {
        // if (bool) {
            // colorServo.setPosition(0.65f);
            // while (colorServo.getPosition() < 0.65f) {}
            // colorServo2.setPosition(0.2f);
            // while (colorServo2.getPosition() > 0.2f) {}
            // colorServo.setPosition(0.8f);
            // while (colorServo.getPosition() < 0.8f) {}
            // return;
        // }
        // colorServo.setPosition(0.65f);
        // while (colorServo.getPosition() > 0.65f) {}
        // colorServo2.setPosition(1f);
        // while (colorServo2.getPosition() < 1f) {}
        // colorServo.setPosition(0.15f);
        // return;
    // }

    // configure the drive train to work in the autonomous period
    public void configAutoPeriod() {
        left_front.setDirection(DcMotor.Direction.FORWARD);
        right_front.setDirection(DcMotor.Direction.FORWARD);
        left_back.setDirection(DcMotor.Direction.FORWARD);
        right_back.setDirection(DcMotor.Direction.FORWARD);
    }

    // function to make the drive train use or not use the encoders
    // (encoders are used for the autonomous periodm otherwise
    // we don't use encoders)
    public void useEncoder(boolean x) {
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (x) {
            left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return;
        }
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return;
    }
 }

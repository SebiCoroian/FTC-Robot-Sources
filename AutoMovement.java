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

import org.firstinspires.ftc.teamcode.RobotHw;

public class AutoMovement {

    // this is the hardware that we're controlling
    RobotHw robot = new RobotHw();

    // number of encoder ticks a full rotation of a wheel has
    public static final int ROTATION = 1120;

    // values for the current value of each encoder of the drive train
    public int left_front_encoder;
    public int left_back_encoder;
    public int right_front_encoder;
    public int right_back_encoder;

    // this is the constructor function, we don't use it
    public AutoMovement() {

    }

    // initialize the autonomous movement by providing the hardware
    public void init(RobotHw r) {
        robot = r;
        left_front_encoder = robot.left_front.getCurrentPosition();
        left_back_encoder = robot.left_back.getCurrentPosition();
        right_front_encoder = robot.right_front.getCurrentPosition();
        right_back_encoder = robot.right_back.getCurrentPosition();
    }

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

        // while ( robot.left_front.isBusy() && robot.left_back.isBusy() && robot.right_front.isBusy() && robot.right_back.isBusy() ) {
        //     //idle();
        // }
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
    public void stop() {
        setPower(0f);
        // refresh();
    }
    
    public void waitToFinish() {
        while ( robot.left_front.isBusy() && robot.right_front.isBusy() && robot.left_back.isBusy() && robot.right_back.isBusy() ) {
            
        }
    }
}

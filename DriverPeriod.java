package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHw;


@TeleOp(name="Driving", group="Brand New")

public class DriverPeriod extends LinearOpMode {

    // the hardware this op mode is going to use
    RobotHw robot = new RobotHw();

    // time passed from the beggining of the op mode
    private ElapsedTime runtime = new ElapsedTime();

    // variables used for collecting glyphs
    private boolean x = false;
    private boolean y = false;
    private boolean safety = true;

    // variable used for scaling up or down the power fed to the
    // drive train motors
    private float powerMultiplier = 1f;

    // function used for collecting qubes
    // place this in the main op mode loop
    private void qubeIntake() {
         if (gamepad2.x == true && safety == true) {
         //de pe reddit
         x = !x;
         safety = false;
        }
         if (gamepad2.y == true && safety == true){
         y = !y;
         safety = false;
        }
        if (gamepad2.x == false){
            safety = true;
        }
        if (gamepad2.y == false){
            safety = true;
        }
        if (gamepad2.y == true){
            robot.left_qubes.setPower(1f);
            robot.right_qubes.setPower(-1f);
        }
        else if (gamepad2.x == true){
            robot.left_qubes.setPower(-1f);
            robot.right_qubes.setPower(1f);
        }
        else {
            robot.left_qubes.setPower(0);
            robot.right_qubes.setPower(0);
        }
    }

    // function used for controlling the ramp used to
    // place the glyphs in the cryptobox
    // place this in the main op mode loop
    private void qubeRamp() {
        if (gamepad2.right_bumper!=false)
        {
            robot.ramp1.setPosition(0.97f);
            robot.ramp2.setPosition(0.97f);
        }
        if (gamepad2.left_bumper!=false)
        {
            robot.ramp1.setPosition(0.13f);
            robot.ramp2.setPosition(0.13f);
        }
    }

    // function used for controlling the lift that elevates
    // the glyphs which allows placing the glyphs in the upper region
    // of the cryptobox
    // place this in the main op mode loop
    private void qubeLift() {
        if (gamepad2.left_trigger > 0) {
            robot.lift.setPower(-gamepad2.left_trigger);
            return;
        }
        if (gamepad2.right_trigger > 0) {
            robot.lift.setPower(gamepad2.right_trigger);
            return;
        }
        robot.lift.setPower(0);
        return;
    }

    // function used for controlling the robotic arms used for manipulating
    // the relic
    // place this in the main op mode loop
    public void relicLift() {
        if (gamepad2.a) {
            robot.relicServo.setPower(1f);
            // sleep(200);
            // robot.relicServo.setPower(0.5);
        }
        if (gamepad2.b) {
            robot.relicServo.setPower(-1f);
            // sleep(200);
            // robot.relicServo.setPower(0.5);
        }
        robot.relic.setPower(gamepad2.right_stick_x);
    }

    // function used for controlling the drive train(making the robot move)
    // place this in the main op mode loop
    private void movement(){
        // the letter buttons on the gamepad change the power multiplier
        // this allows switching the robot speed in order to make it easier
        // to drive in special situations
        if (gamepad1.x == true) {
            powerMultiplier = 1f;
        }
        if (gamepad1.y == true) {
            powerMultiplier = 0.75f;
        }
        if (gamepad1.b == true) {
            powerMultiplier = 0.5f;
        }
        if (gamepad1.a == true) {
            powerMultiplier = 0.35f;
        }

        // if the robot has to rotate(the right stick x value is diffrent
        // from zero), make it rotate and ignore other actions
        if (gamepad1.right_stick_x != 0){
            robot.left_back.setPower(-gamepad1.right_stick_x*powerMultiplier);
            robot.left_front.setPower(-gamepad1.right_stick_x*powerMultiplier);
            robot.right_back.setPower(-gamepad1.right_stick_x*powerMultiplier);
            robot.right_front.setPower(-gamepad1.right_stick_x*powerMultiplier);
            return;
        }

        // otherwise, if the robot has to move(the left stick x and y values
        // are diffrent from zero), make it move and ignore other actions
        if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
            float x = gamepad1.left_stick_x * powerMultiplier;
            float y = -gamepad1.left_stick_y * powerMultiplier;
            robot.left_front.setPower(-y-x);
            robot.right_front.setPower(y-x);
            robot.right_back.setPower(y+x);
            robot.left_back.setPower(-y+x);
            return;
        }

        // otherwise, make the robot stop
        robot.left_front.setPower(0);
        robot.right_front.setPower(0);
        robot.right_back.setPower(0);
        robot.left_back.setPower(0);
        return;
    }

    // function called when the coach hits init
    @Override
    public void runOpMode() {

        // initialize the hardware(see RobotHw class for more details)
        robot.init(hardwareMap);
        // configure the drive train for the driver period
        robot.configDriverPeriod();
        // disable the usage of encoders for the drive train
        robot.useEncoder(false);

        // lift the color servo so that it doesn't hang
        robot.colorServo.setPosition(0f);

        // wait until the coach hits play
        waitForStart();
        // reset the runtime
        runtime.reset();

        // enter the main op mode lopp
        // this stops when the coach hits stop
        while (opModeIsActive()) {

            // move the robot
            movement();

            // collect glyphs
            qubeIntake();

            // manipulate glyphs
            qubeRamp();
            qubeLift();

            if (gamepad2.dpad_left == true) {
                robot.colorServo.setPosition(0.15f);
            }
            if (gamepad2.dpad_right == true) {
                robot.colorServo.setPosition(0.89f);
            }

            // manipulate the relic arm(this is WIP so it's disabled)
            relicLift();

            // show in the telemetry area the elapsed time from the moment
            // the driver hits play
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

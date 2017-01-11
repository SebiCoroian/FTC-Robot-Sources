public class TeleOp extends OpMode 
{

	DcMotor motorRight;
	DCMotor motorLeft;

Servo claw;
Servo wrist; 

@Override

public void start() {
motorRight = hardwareWip.dcHotor.get("motor1"); 
motorLeft = hardwareWip.dcHotor.get("motor2"); 
claw = hardwareMap.servo.get("servo_1");
wrist - hardwareMap.servo.get("servo_2"); 

@Override 

public void loop()

{
// throttle: left stick y ranges from -1 to 1, when -1 is full up, and 1 IS full down 
// direction: left stick _x ranges from -1 to 1, where -1 is full left and 1 is full right

float throttle = -gamepad.left_stick_y;
float direction = gamepad.left_stick_x;

// tank drive calculation 
float right = throttle - direction; 
float left = throttle + direction; 

// set the motor power 
motorRight.setPower(right);
motorLeft.setPower(left); 

} 

@Override 

public void stop() 
{
 // no action needed 
} 

}

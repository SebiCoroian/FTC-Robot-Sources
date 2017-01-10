public void loop()

{
//Is an IR signed detected?
if (irSeeker.signalDetected())

{

// an IR signal is detected
double angle = irSeeker.getAngle();
double strength = irSeeter.getAngle();

//which direction should we move?
if (angle < 0) 
{

//we need to move to the left
motorRight.setPower(MOTOR_POWER); 
motorLeft.setPower(-MOTOR_POWER); 

}
 
else if (angle > 0)
{
//we need to move to the right
motorRight.setPower(-MOTOR_POWER);
motorLeft.setPower(MOTOR_POWER);
}
else if (strength <HOLD_IR_SIGNAL_STRENGTH)
{
// the IR signal is weak, approach
motorRight.setPower(MOTOR_POWER);
motorLeft.setPower(MOTOR_POWER);
} 
else 
{
// the IR signal Is strong, stay here
motorRight.setPower(0.0);
motorLeft.setPower(0.0);
}
 
}
else
{
//no IR signal is detected
motorRight.setPower(0.0);
motorLeft.setPower(0.0);
}

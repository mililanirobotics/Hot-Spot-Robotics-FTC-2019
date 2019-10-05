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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="mecanumDrive", group="Iterative Opmode")

public class mecanumDrive extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    /*private DcMotor intakeOne = null;
    private DcMotor intakeTwo = null;
    private DcMotor pulley = null;*/

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftBack  = hardwareMap.get(DcMotor.class, "left_Back"); // Green, Violet
        rightBack = hardwareMap.get(DcMotor.class, "right_Back"); // Green, Blue
        leftFront  = hardwareMap.get(DcMotor.class, "left_Front"); // Brown, Blue
        rightFront = hardwareMap.get(DcMotor.class, "right_Front"); //Brown, Violet
        /*intakeOne = hardwareMap.get(DcMotor.class, "intake_One");
        intakeTwo = hardwareMap.get(DcMotor.class, "intake_Two");
        pulley = hardwareMap.get (DcMotor.class, "Lift");*/

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        /*intakeOne.setDirection(DcMotor.Direction.FORWARD);
        intakeTwo.setDirection(DcMotor.Direction.FORWARD);
        pulley.setDirection (DcMotor.Direction.FORWARD);*/

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    //This function converts the points from the joysticks to degrees
    public double convertToPolarAngle (double x, double y)
    {
        double degree = (-((Math.atan2(y, x))* 180/Math.PI)) + 90;
        if (degree > 180){
            degree -= 360;
        }
        return degree;
    }
    //This funtion converts the dgrees into the power needed for the motors
    public void motorPower(double angle, double forwardPower, double sidePower, double turnPower)
    {
        double threshold = 0.05;//This is used so that the input has to be greater than the threshold so movements not wanted don't happen
        forwardPower = -forwardPower; // this variable is used to state the power needed when the y axis of joystick is moved
       if(Math.abs(forwardPower) >= threshold && Math.abs(sidePower) < threshold && Math.abs(turnPower) < threshold)//checks to make sure that the y axis on the joystick is the only one getting power
       {
           leftBack.setPower(forwardPower);//sets the powers to the motors
           rightBack.setPower(forwardPower);
           leftFront.setPower(forwardPower);
           rightFront.setPower(forwardPower);
       }
       else if(Math.abs(forwardPower) < threshold && Math.abs(sidePower) >= threshold && Math.abs(turnPower) < threshold)//checks to make sure that the x axis on the joystick is the only one getting power
       {
           leftBack.setPower(-sidePower);
           rightBack.setPower(sidePower);
           leftFront.setPower(sidePower);
           rightFront.setPower(-sidePower);
       }
       else if(Math.abs(forwardPower) < threshold && Math.abs(sidePower) < threshold && Math.abs(turnPower) >= threshold)//checks to make sure that the y axis on the joystick on thr right side is the only one getting power
       {
           leftBack.setPower(turnPower);
           rightBack.setPower(-turnPower);
           leftFront.setPower(turnPower);
           rightFront.setPower(-turnPower);
       }
       else{
           // degrees is proportional to power / powerScale
           double leftFrontPower = forwardPower + sidePower + turnPower;
           double rightFrontPower = -forwardPower + sidePower + turnPower;
           double rightBackPower = -forwardPower - sidePower + turnPower;
           double leftBackPower = forwardPower - sidePower + turnPower;
           double powerScale = 3;
           if (angle > 0 && angle < 45)
           {
               powerScale = 1 + Math.abs(forwardPower/sidePower) + Math.abs(turnPower);
           }
           else if (angle >= 45 && angle < 90)
           {
               powerScale = 1 + Math.abs(sidePower/forwardPower) + Math.abs(turnPower);
           }
           else if (angle > 90 && angle < 135)
           {
               powerScale = 1 + Math.abs(forwardPower/sidePower) + Math.abs(turnPower);
           }
           else if (angle >= 135 && angle < 180)
           {
               powerScale = 1 + Math.abs(sidePower/forwardPower) + Math.abs(turnPower);
           }
           else if (angle >= -179 && angle < -135)
           {
               powerScale = 1 + Math.abs(forwardPower/sidePower) + Math.abs(turnPower);
           }
           else if (angle >= -135 && angle < -90)
           {
               powerScale = 1 + Math.abs(sidePower/forwardPower) + Math.abs(turnPower);
           }
           else if (angle > -90 && angle < -45)
           {
               powerScale = 1 + Math.abs(forwardPower/sidePower) + Math.abs(turnPower);
           }
           else if (angle >= -45 && angle < 0)
           {
               powerScale = 1 + Math.abs(sidePower/forwardPower) + Math.abs(turnPower);
           }
           leftBack.setPower(leftBackPower/powerScale);
           rightBack.setPower(rightBackPower/powerScale);
           leftFront.setPower(leftFrontPower/powerScale);
           rightFront.setPower(rightFrontPower/powerScale);
       }
    }


    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double forwardPower;
        double sidePower;
        double turnPower;
        double liftpower;


        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double forwardBackward = gamepad1.left_stick_y;
        double sideWays = gamepad1.left_stick_x;
        double turn  =  gamepad1.right_stick_x;
        double liftMechanism = gamepad2.left_stick_y;

        forwardPower = Range.clip(forwardBackward, -1.0,1.0);
        sidePower = Range.clip(sideWays, -1.0, 1.0);
        turnPower = Range.clip(turn, -1.0, 1.0);
        liftpower = Range.clip(liftMechanism, -1.0, 1.0);

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;
        /*if (gamepad2.right_bumper == true)
        {
            intakeOne.setPower(0.3);
            intakeTwo.setPower(0.3);
        }
        pulley.setPower(liftpower);
        intakeOne.setPower(0);
        intakeTwo.setPower(0);*/
        double degrees = convertToPolarAngle(sideWays, forwardBackward);
        motorPower(degrees, forwardPower, sidePower, turnPower);

        // Send calculated power to wheels for turn


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "degrees (%.2f)", degrees);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

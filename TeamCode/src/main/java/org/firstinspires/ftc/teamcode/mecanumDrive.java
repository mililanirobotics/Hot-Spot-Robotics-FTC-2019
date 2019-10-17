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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

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
    public final static double heightOfLift = 11; // This variable is a static variable that holds the height of the lift in inches

    private ElapsedTime runtime = new ElapsedTime();

    // This code is to set the DcMotors to a variable
    private DcMotor leftBack = null; // The left back motor
    private DcMotor rightBack = null; // The right back motor
    private DcMotor leftFront = null; // The left front motor
    private DcMotor rightFront = null; // the right front motor
    private DcMotor intakeOne = null; // This is for the other motor for the intake system
    private DcMotor intakeTwo = null; // This is one of the motors for the intake system
    private DcMotor pulley = null; // This is for the lift
    private DcMotor swing = null; //This is the claw motor that swings the claw

    // This code is to set the Servos to a variable
    private Servo claw; // For the claw

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
        intakeOne = hardwareMap.get(DcMotor.class, "intake_One"); //
        intakeTwo = hardwareMap.get(DcMotor.class, "intake_Two");
        pulley = hardwareMap.get (DcMotor.class, "Lift");
        swing = hardwareMap.get(DcMotor.class, "swing_Claw");


        claw = hardwareMap.get(Servo.class, "claw");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        intakeOne.setDirection(DcMotor.Direction.FORWARD);
        intakeTwo.setDirection(DcMotor.Direction.FORWARD);
        pulley.setDirection (DcMotor.Direction.FORWARD);
        swing.setDirection(DcMotor.Direction.FORWARD);

        //This is to set the motor to reset the encoders in the motor
        pulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // This is to state to use the encoder
        pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        if(Math.abs(forwardPower) > 0.05 && Math.abs(turnPower) > 0.05 && Math.abs(sidePower) < 0.05)
        {
            if(turnPower > 0)
            {
                leftBack.setPower(forwardPower);
                leftFront.setPower(forwardPower);
                rightBack.setPower(0);
                rightFront.setPower(0);
            }
            else if(turnPower < 0)
            {
                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(forwardPower);
                rightFront.setPower(forwardPower);
            }

        }
        else
            {
            double leftFrontPower;
            double rightFrontPower;
            double leftBackPower;
            double rightBackPower;
            double power;

            double radians = (angle * (Math.PI / 180)); //This is to the radians out of the degrees to use the equation
            if (forwardPower >= sidePower) {
                power = (forwardPower); // To determine the power it should use
            } else if (forwardPower <= sidePower) {
                power = (sidePower); // To determine the power it should use
            } else {
                power = 0;
            }

            leftFrontPower = (power * Math.sin(radians + (Math.PI / 4)));
            rightFrontPower = (power * Math.cos(radians + (Math.PI / 4)));
            leftBackPower = (power * Math.cos(radians + (Math.PI / 4)));
            rightBackPower = (power * Math.sin(radians + (Math.PI / 4)));
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
        }

    }



    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double forwardPower;
        double sidePower;
        double turnPower;
        double liftpower;
        double swingPower;


        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double forwardBackward = gamepad1.left_stick_y;
        double sideWays = gamepad1.left_stick_x;
        double turn  =  gamepad1.right_stick_x;
        double liftMechanism = gamepad2.left_stick_y;
        double swingClaw = gamepad2.right_stick_x;

        forwardPower = Range.clip(forwardBackward, -1.0,1.0);
        sidePower = Range.clip(sideWays, -1.0, 1.0);
        turnPower = Range.clip(turn, -1.0, 1.0);
        liftpower = Range.clip(liftMechanism, -1.0, 1.0);
        swingPower = Range.clip(swingClaw,-1.0, 1.0);

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;


        //calls upon the function of degrees to return the degrees of the left joystick on controller 1
        double degrees = convertToPolarAngle(sideWays, forwardBackward);

        // calls upon the function  motorpower to set the powers of the motor based on the input of your joysticks on your controller 1
        motorPower(degrees, forwardPower, sidePower, turnPower);

        // this sets the power of the swing motor to the value of the right joystick on controller 2
        swing.setPower(swingPower);

        // This sets the power of the lift motor to the value of the left joystick on controller 2
        pulley.setPower(liftpower);

        // This checks to see if the right bumper is hit. If so then the power is set to 1 for the intake motors
        if (gamepad2.right_bumper == true)
        {
            intakeOne.setPower(1);
            intakeTwo.setPower(1);

        }

        //sets the power to 0 so that way when the right bumper is no longer pushed the motors stop
        intakeOne.setPower(0);
        intakeTwo.setPower(0);


        // When the right trigger is slightly pressed the clow will close up
        if (gamepad2.right_trigger >= .1)
        {
            claw.setPosition(0);
        }
        // When the left trigger is slightly pressed the claw will go out
        else if(gamepad2.left_trigger >= .1)
        {
            claw.setPosition(1);
        }


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

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
import com.qualcomm.robotcore.hardware.Servo;
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

@TeleOp(name="Tank", group="Iterative Opmode")

public class Tank extends OpMode {
    // Declare OpMode members.
    public final static double heightOfLift = 11; // This variable is a static variable that holds the height of the lift in inches

    private ElapsedTime runtime = new ElapsedTime();

    // This code is to set the DcMotors to a variable
    private DcMotor leftBack = null; // The left back motor
    private DcMotor rightBack = null; // The right back motor
    private DcMotor leftFront = null; // The left front motor
    private DcMotor rightFront = null; // the right front motor
    private DcMotor pulley = null; // This is for the lift
    private DcMotor clawLift = null;

    // This code is to set the Servos to a variable
     private Servo claw;  //For the claw*/
    private Servo foundationOne;
    private Servo foundationTwo;


    double slowMode = 1;

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
        pulley = hardwareMap.get (DcMotor.class, "Lift");
        clawLift = hardwareMap.get(DcMotor.class, "claw_Lift");


        claw = hardwareMap.get(Servo.class, "Claw");
        foundationOne = hardwareMap.get(Servo.class, "foundation_One");
        foundationTwo = hardwareMap.get(Servo.class, "foundation_Two");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        pulley.setDirection (DcMotor.Direction.FORWARD);
        clawLift.setDirection(DcMotor.Direction.FORWARD);

        claw.setPosition(0);
        foundationOne.setPosition(0);
        foundationTwo.setPosition(0);

        //This is to set the motor to reset the encoders in the motor
        pulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // This is to state to use the encoder
        pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
    public double driveAngle(double x, double y) {
        double degree = (((Math.atan2(y, x)) * 180 / Math.PI)) + 90;
        if (degree > 180) {
            degree -= 360;
        }
        return degree;
    }
    //This function converts the degrees into the power needed for the motors
    public void motorPower(double angle, double forwardPower, double sidePower, double turnPower, double diviser)
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
        else if (Math.abs(turnPower) > 0.05 && Math.abs(forwardPower) < 0.05 && Math.abs(sidePower) < 0.05)
        {
            leftFront.setPower(turnPower);
            leftBack.setPower(turnPower);
            rightFront.setPower(-turnPower);
            rightBack.setPower(-turnPower);
        }
        else
            {
            double leftFrontPower;
            double rightFrontPower;
            double leftBackPower;
            double rightBackPower;
            double power;

            double radians = (angle * (Math.PI / 180)); //This is to the radians out of the degrees to use the equation
            
            if (Math.abs(forwardPower) >= Math.abs(sidePower)) {
                power = (Math.abs(forwardPower)); // To determine the power it should use
            } else if (Math.abs(forwardPower) <= Math.abs(sidePower)) {
                power = (Math.abs(sidePower)); // To determine the power it should use
            } else {
                power = 0;
            }

            leftFrontPower = (power * Math.sin(radians + (Math.PI / 4)));
            rightFrontPower = (power * Math.cos(radians + (Math.PI / 4)));
            leftBackPower = (power * Math.cos(radians + (Math.PI / 4)));
            rightBackPower = (power * Math.sin(radians + (Math.PI / 4)));
            
            leftFrontPower = (leftFrontPower/diviser);
            rightFrontPower = (rightFrontPower/diviser);
            leftBackPower = (leftBackPower/diviser);
            rightBackPower = (rightBackPower/diviser);
            
            leftFront.setPower(-leftFrontPower);
            rightFront.setPower(-rightFrontPower);
            leftBack.setPower(-leftBackPower);
            rightBack.setPower(-rightBackPower);
        }

    }


    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double rightSidePower;
        double leftSidePower;
        double liftpower;
        double clawPower;


        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double left = gamepad1.left_stick_y;
        double leftSide = gamepad1.left_stick_x;
        double right = gamepad1.right_stick_y;
        double rightSide = gamepad1.right_stick_x;
        double liftMechanism = gamepad2.left_stick_y;
        double pivot = gamepad2.right_stick_x;


        leftPower = Range.clip(left, -1.0, 1.0);
        rightPower = Range.clip(right, -1.0, 1.0);
        leftSidePower = Range.clip(leftSide, -1.0, 1.0);
        rightSidePower = Range.clip(rightSide, -1.0, 1.0);
        liftpower = Range.clip(liftMechanism, -1.0, 1.0);
        clawPower = Range.clip(pivot, -1, 1);

        if (Math.abs(leftPower) >= .05) {
            leftFront.setPower(leftPower);
            leftBack.setPower(leftPower);
        }
        if (Math.abs(rightPower) >= .05)
        {
            rightFront.setPower(rightPower);
            rightBack.setPower(rightPower);
        }
        if (Math.abs(leftSidePower) >= .05)
        {
            leftFront.setPower(leftPower);
            leftBack.setPower(-leftPower);
        }
        if(Math.abs(rightSidePower) >= 0.5)
        {
            rightFront.setPower(-rightPower);
            rightBack.setPower(rightPower);
        }
        else if(Math.abs(rightPower) <= 0.05 && Math.abs(leftPower) <= 0.05 && Math.abs(rightSidePower) <= 0.05 && Math.abs(leftSidePower) <= 0.05){
            rightFront.setPower(0);
            rightBack.setPower(0);
            leftFront.setPower(0);
            leftBack.setPower(0
            );
        }



        /*if (gamepad1.left_bumper == true)
        {
            slowMode = 1;
        }

        if (gamepad1.right_bumper == true)
        {
            slowMode = 2;

        }*/



            // this sets the power of the swing motor to the value of the right joystick on controller 2

            // This sets the power of the lift motor to the value of the left joystick on controller 2
        pulley.setPower(liftpower);
        if (Math.abs(liftpower) < 0.05)
        {
            pulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (gamepad2.a == true) {
            foundationOne.setPosition(1);
            foundationTwo.setPosition(1);
        }
        else if (gamepad2.b == true) {
            foundationOne.setPosition(0);
            foundationTwo.setPosition(0);
        }

        clawLift.setPower(clawPower);

            // This checks to see if the right bumper is hit. If so then the power is set to 1 for the intake motors


            // When the right trigger is slightly pressed the claw will close up
        if (gamepad2.right_bumper == true)
        {
            claw.setPosition(.5);
        }
            // When the left trigger is slightly pressed the claw will go out
        if (gamepad2.left_bumper == true) ;
        {
            claw.setPosition(0);
        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "slowmode: (%.d)", slowMode);
        //telemetry.addData("Encoders", "leftFront: (%.d), rightFront: (%.d), leftBack: (%.d), rightBack: (%.d)", leftFront.getCurrentPosition(), rightFront.getCurrentPosition(), leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
    }

}


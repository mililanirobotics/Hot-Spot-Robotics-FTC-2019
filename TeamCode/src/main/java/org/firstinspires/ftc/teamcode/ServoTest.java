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

@TeleOp(name="ServoTest", group="Iterative Opmode")

public class ServoTest extends OpMode {
    // Declare OpMode members.
    //public final static double heightOfLift = 11; // This variable is a static variable that holds the height of the lift in inches

    private ElapsedTime runtime = new ElapsedTime();

    // This code is to set the DcMotors to a variable

    // This code is to set the Servos to a variable
     private Servo claw;  //For the claw*/
    //private Servo foundationOne;
    //private Servo foundationTwo;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        claw = hardwareMap.get(Servo.class, "Claw");
        //foundationOne = hardwareMap.get(Servo.class, "foundation_One");
        //foundationTwo = hardwareMap.get(Servo.class, "foundation_Two");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

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
        double turn = gamepad1.right_stick_x;
        double liftMechanism = gamepad2.left_stick_y;



        forwardPower = Range.clip(forwardBackward, -1.0, 1.0);
        sidePower = Range.clip(sideWays, -1.0, 1.0);
        turnPower = Range.clip(turn, -1.0, 1.0);
        liftpower = Range.clip(liftMechanism, -1.0, 1.0);

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


            /*if (gamepad2.a == true) {
                foundationOne.setPosition(0.5);
                foundationTwo.setPosition(0.5);
            } else if (gamepad2.b == true) {
                foundationOne.setPosition(0);
                foundationTwo.setPosition(0);
            }*/

            // This checks to see if the right bumper is hit. If so then the power is set to 1 for the intake motors


            // When the right trigger is slightly pressed the claw will close up
        if (gamepad2.right_bumper == true)
        {

            claw.setPosition(.4);
        }
            // When the left trigger is slightly pressed the claw will go out
        else if (gamepad2.left_bumper == true)
        {
            claw.setPosition(1);
        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

}


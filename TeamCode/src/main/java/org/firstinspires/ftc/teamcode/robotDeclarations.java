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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;


public class robotDeclarations
{
    // This is to assign the function of the DcMotor to the variables
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftBack = null; // drive Motors
    public DcMotor rightBack = null; // drive Motors
    public DcMotor leftFront = null; // drive Motors
    public DcMotor rightFront = null; // drive Motors
    public DcMotor pulley = null; // lift Motor
    private DcMotor clawLift = null;

    // This is to assign the function of the servo to the variable
    public Servo claw; // claw servo
    private Servo foundationOne;
    private Servo foundationTwo;

    // This is to assign the function of the color sensor to the the variable



    /* local OpMode members. */
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public robotDeclarations(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hardwareMap) {

        // Define and Initialize Motors
        leftBack  = hardwareMap.get(DcMotor.class, "left_Back"); // Green, Violet
        rightBack = hardwareMap.get(DcMotor.class, "right_Back"); // Green, Blue
        leftFront  = hardwareMap.get(DcMotor.class, "left_Front"); // Brown, Blue
        rightFront = hardwareMap.get(DcMotor.class, "right_Front"); //Brown, Violet
        pulley = hardwareMap.get (DcMotor.class, "Lift");
        clawLift = hardwareMap.get(DcMotor.class, "claw_Lift");


        // Defines and Initialize the Servo
        claw = hardwareMap.get(Servo.class, "Claw");
        foundationOne = hardwareMap.get(Servo.class, "foundation_One");
        foundationTwo = hardwareMap.get(Servo.class, "foundation_Two");


        // Changes the direction of the motor if the motor is reversed
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);


        // Resets the encoder so that the ticks counted is back to 0
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set all motors to zero power
        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        pulley.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

    }
 }


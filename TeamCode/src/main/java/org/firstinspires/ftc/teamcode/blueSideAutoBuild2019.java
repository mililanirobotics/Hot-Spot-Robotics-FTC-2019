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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="blueSideAutoBuild2019", group="Pushbot")

public class blueSideAutoBuild2019 extends LinearOpMode {

    /* Declare OpMode members. */
    robotDeclarations robot   = new robotDeclarations();// Use is calling upon the file called robotDeclarations where all the devices are init

    private ElapsedTime     runtime = new ElapsedTime(); // For the use of time

    final static int DIAMETER                = 4; // This is the diameter of the wheels of the robot in inches

    final static double CIRCUMFRENCE         = Math.PI * DIAMETER; // calculates the circumference of the wheels in inches
    final static double GEARREDUCTION        = 1; // If we were to have gears the gear reduction would go up or down depending
    final static int TICKSPERROTATION        = 560; // there is 1680 ticks per a revolution of a motor
    public final static double ROBOTRADIOUS  = 7.5; // the radious of the robot is 7.5 inches
    public final static double TILELENGTH    = 24; // the length of a tile is 24 inches or 2 feet
    public final static double TICKSPERANINCHSTRAFING = 37;


    // This is needed for the vuforia licence that we use for image detection


    //determins the amount of ticks per an inch
    static final double COUNTS_PER_INCH = (TICKSPERROTATION * GEARREDUCTION) / CIRCUMFRENCE;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive syst
         *
         *  em variables.
         * The init() method of the hardware class does all the work here
         */

        robot.init(hardwareMap);


            // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          robot.leftFront.getCurrentPosition(),
                          robot.rightFront.getCurrentPosition());
        telemetry.update();


        // This is getting the amount of inches the wheel has to turn to reach 90 degrees
        double inchesMoved = degreesToInches(90);
        robot.foundationOne.setPosition(0);
        robot.foundationTwo.setPosition(1);



        //calls upon the encoderDrive function which converts the amount of inches into ticks that the encoder should be set to and sets the power to move the robot
        //In this code the time out is meant to be a fail safe incase somthing goes wrong the code will timeout so it does not just keep going

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        strafedrive(1, -20 * TICKSPERANINCHSTRAFING, 3);
        encoderDrive(.5, 28, 28, 3);
        robot.foundationOne.setPosition(1);
        robot.foundationTwo.setPosition(0);
        try {
            TimeUnit.SECONDS.sleep(1);
        }
        catch (InterruptedException e){

        }
        encoderDrive(.3, -40, -40, 8);
        robot.foundationOne.setPosition(0);
        robot.foundationTwo.setPosition(1);
        try {
            TimeUnit.SECONDS.sleep(1);
        }
        catch (InterruptedException e){

        }



        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    public void strafedrive(double speed,
                             double amountOfInches,
                             double timeoutS) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.leftFront.setTargetPosition((int)(-amountOfInches));
            robot.rightFront.setTargetPosition((int)(amountOfInches));
            robot.leftBack.setTargetPosition((int)(amountOfInches));
            robot.rightBack.setTargetPosition((int)(-amountOfInches));

            // Turn On RUN_TO_POSITION
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftBack.isBusy() && robot.rightBack.isBusy() && robot.leftFront.isBusy() && robot.rightFront.isBusy())) {

                if(amountOfInches < 0)
                {
                    robot.leftBack.setPower(-speed);
                    robot.leftFront.setPower(speed);
                    robot.rightBack.setPower(speed);
                    robot.rightFront.setPower(-speed);
                }
                else if(amountOfInches > 0)
                {
                    robot.leftBack.setPower(speed);
                    robot.leftFront.setPower(-speed);
                    robot.rightBack.setPower(-speed);
                    robot.rightFront.setPower(speed);
                }
                // Display it for the driver.
                telemetry.addData("Strafe",  "Running at %7d :%7d",
                        robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.leftFront.setTargetPosition((int)(leftInches * COUNTS_PER_INCH));
            robot.rightFront.setTargetPosition((int)(rightInches * COUNTS_PER_INCH));
            robot.leftBack.setTargetPosition((int)(leftInches * COUNTS_PER_INCH));
            robot.rightBack.setTargetPosition((int)(rightInches * COUNTS_PER_INCH));

            // Turn On RUN_TO_POSITION
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftBack.isBusy() && robot.rightBack.isBusy() && robot.leftFront.isBusy() && robot.rightFront.isBusy())) {

                if(leftInches < 0 && rightInches < 0)
                {
                    robot.leftBack.setPower(-speed);
                    robot.leftFront.setPower(-speed);
                    robot.rightBack.setPower(-speed);
                    robot.rightFront.setPower(-speed);
                }
                else if(leftInches > 0 && rightInches > 0)
                {
                    robot.leftBack.setPower(speed);
                    robot.leftFront.setPower(speed);
                    robot.rightBack.setPower(speed);
                    robot.rightFront.setPower(speed);
                }
                // Display it for the driver.
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.leftFront.getCurrentPosition(),
                                            robot.rightFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public double degreesToInches(double degrees)
    {
        double radians = (degrees*(Math.PI/180));
        double inchesMove = (radians * ROBOTRADIOUS);
        return(inchesMove);
    }

}

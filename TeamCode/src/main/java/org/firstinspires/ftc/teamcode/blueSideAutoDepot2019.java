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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robotDeclarations;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

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

@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")

public class blueSideAutoDepot2019 extends LinearOpMode {

    /* Declare OpMode members. */
    robotDeclarations robot   = new robotDeclarations();// Use is calling upon the file called robotDeclarations where all the devices are init

    private ElapsedTime     runtime = new ElapsedTime(); // For the use of time

    final static int DIAMETER                = 4; // This is the diameter of the wheels of the robot in inches

    final static double CIRCUMFRENCE         = Math.PI * DIAMETER; // calculates the circumference of the wheels in inches
    final static double GEARREDUCTION        = 1; // If we were to have gears the gear reduction would go up or down depending
    final static int TICKSPERROTATION        = 1680; // there is 1680 ticks per a revolution of a motor
    public final static double ROBOTRADIOUS  = 7.5; // the radious of the robot is 7.5 inches
    public final static double TILELENGTH    = 24; // the length of a tile is 24 inches or 2 feet

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite"; // This is to define a variable of the model assets
    private static final String LABEL_FIRST_ELEMENT = "Stone"; // This defines a variable for a stone for tensorflow
    private static final String LABEL_SECOND_ELEMENT = "Skystone"; // This defines a variable to a skystone for tensorflow


    // This is needed for the vuforia licence that we use for image detection
    private static final String VUFORIA_KEY = "ARqoPXv/////AAABmZ9oE4NYn0cXsd928uU8EgMXcL9ps686/WyyudSMh8i+yq2a2P0udLUULuHoFvf2Ibcar1hHvdKyQDij94K3L36FbSp072xjzhvMAwGrbjBJd/1qAEVhaaJoeYl2ofDROH/LCMPwWg39fthIvMjAiv9JiFE+h/D9V0ocepY1I/8eGQH3dtq43QlLMlNWoz/zrP7wHQvIeB0XIYtjsWtz/ruz/542uajyWmd515SMl2Xpd5klGgjaQbbY7KJ8BotR7q45Jy1QTOvFjQ4UFFrb9+keNG3KEjCHGmn+95q+GEj14ThvysgFd9yVFvCTZmsHo1/d62/xkVlls6dLyleJCwImIKMHwcUp+l5NH2xxAFAR";

    private VuforiaLocalizer vuforia; // creates a variable to import the VuforiaLocalizer into a word more usable

    private TFObjectDetector tfod; // creates a variable to import the TFObjectDetector into a word more usable


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

        // Init the vuforia
        initVuforia();


        // This is to check if the phone can use TensorFlow for the image recognition if it can't then the error pops up saying device is not compatible
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }





            // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        // Resets the encoders back to 0 for the ticks that it has calulated
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Sets the motors to run using the encoders
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          robot.leftFront.getCurrentPosition(),
                          robot.rightFront.getCurrentPosition());
        telemetry.update();


        // This is getting the amount of inches the wheel has to turn to reach 90 degrees
        double inchesMoved = degreesToInches(90);

        // Robot color sensors stating the values that the color sensor detected at the time
        telemetry.addData("Color", "blue: %7d red: %7d", robot.color_Sensor.red(), robot.color_Sensor.blue());


        //calls upon the encoderDrive function which converts the amount of inches into ticks that the encoder should be set to and sets the power to move the robot
        //In this code the time out is meant to be a fail safe incase somthing goes wrong the code will timeout so it does not just keep going
        encoderDrive(1,  inchesMoved,  -inchesMoved, 5.0);
        encoderDrive(1, (TILELENGTH * 1) , (TILELENGTH * 1), 5.0);
        encoderDrive(1,  -inchesMoved,  inchesMoved, 5.0);
        encoderDrive(1, (TILELENGTH * 1) , (TILELENGTH * 1), 5.0);


        encoderDrive(1,  -inchesMoved,  inchesMoved, 5.0);
        encoderDrive(1, (TILELENGTH * 2.5) , (TILELENGTH * 2.5), 5.0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                        }
                        telemetry.update();
                    }
                }
            }
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
    public void pulley (double speed, double pullyInches, double timeoutS)
    {
        int newPulleyTarget;

        if (opModeIsActive())
        {
            newPulleyTarget = robot.leftFront.getCurrentPosition() + (int)(pullyInches * COUNTS_PER_INCH);
            robot.pulley.setTargetPosition(newPulleyTarget);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            robot.leftBack.setPower(Math.abs(speed));
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftBack.isBusy() && robot.rightBack.isBusy() && robot.leftFront.isBusy() && robot.rightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d",  newPulleyTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.pulley.getCurrentPosition());
                telemetry.update();
            }
        }

    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.leftFront.setTargetPosition((int)(leftInches * COUNTS_PER_INCH));
            robot.rightFront.setTargetPosition((int)(rightInches * COUNTS_PER_INCH));
            robot.leftFront.setTargetPosition((int)(leftInches * COUNTS_PER_INCH));
            robot.rightFront.setTargetPosition((int)(rightInches * COUNTS_PER_INCH));

            // Turn On RUN_TO_POSITION
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            if(leftInches < 0)
            {
                robot.leftBack.setPower(-speed);
                robot.leftFront.setPower(-speed);
            }
            else if(rightInches < 0)
            {
                robot.rightBack.setPower(-speed);
                robot.rightFront.setPower(-speed);
            }
            else if(leftInches > 0)
            {
                robot.leftBack.setPower(speed);
                robot.leftFront.setPower(speed);
            }
            else if(rightInches > 0)
            {
                robot.rightBack.setPower(speed);
                robot.rightFront.setPower(speed);
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftBack.isBusy() && robot.rightBack.isBusy() && robot.leftFront.isBusy() && robot.rightFront.isBusy())) {

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
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    private void initTfod()
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}

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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Drive Intake Launcher Power Options OpMode", group="Linear Opmode")

public class DriveIntakeLauncherPower extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    /* DRIVE */
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor centerDrive = null;
    /* INTAKE */
    private DcMotor conveyorBelt = null;

    private CRServo intake = null;

    /* LAUNCHER */
    private DcMotor launchLeft = null;
    private DcMotor launchRight = null;

    /* TRIGGER */
    private Servo Trigger = null;
    private double trigger_extended = 0.76;
    private double trigger_retracted = -0.86;
    private Boolean isPressed = false;

    /*WOBBLE GOAL*/
    private DcMotor linearSlide = null;
    private Servo servo = null;
    private WobbleGoal wobbleGoal = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        /* DRIVE */
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        centerDrive = hardwareMap.get(DcMotor.class, "center_drive");

        /* INTAKE */
        conveyorBelt = hardwareMap.get(DcMotor.class, "belt");

        intake = hardwareMap.get(CRServo.class, "intake");

        /* LAUNCHER */
        launchLeft = hardwareMap.get(DcMotor.class, "launch_left");;
        launchRight = hardwareMap.get(DcMotor.class, "launch_right");;

        /* TRIGGER */
        Trigger = hardwareMap.get(Servo.class, "trigger");

        /* WOBBLE GOAL*/
        linearSlide  = hardwareMap.get(DcMotor.class, "slide");
        servo = hardwareMap.get(Servo.class, "claw");
        wobbleGoal = new WobbleGoal(linearSlide, servo);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        /* DRIVE */
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        centerDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        centerDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /* INTAKE */
        conveyorBelt.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(CRServo.Direction.FORWARD); //Setting up so positive is intaking but negative is pushing the rings away

        /* LAUNCHER */
        launchLeft.setDirection(DcMotor.Direction.REVERSE);
        launchRight.setDirection(DcMotor.Direction.FORWARD);

        /* WOBBLE GOAL */
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            /*DRIVE*/
            double leftPower;
            double rightPower;
            double centerPower;

            /*INTAKE*/
            double conveyorPower = 1;

            double intakePower = 1;

            /*LAUNCHER*/
            double launchLow  = 0.40;
            double launchHigh = 0.45;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            //DRIVE
            double turn = -gamepad1.left_stick_y * .75; //+
            double drive  =  gamepad1.right_stick_x * 0.5;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            centerPower = gamepad1.left_stick_x;

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            centerDrive.setPower(centerPower);

            //INTAKE SYSTEM
            if (gamepad1.a  || gamepad2.right_bumper ){
                intake.setPower(intakePower);
                conveyorBelt.setPower(conveyorPower);
            }
            else if (gamepad1.b || gamepad2.left_bumper){
                intake.setPower(-intakePower);
                conveyorBelt.setPower(-conveyorPower);
            }
            else{
                intake.setPower(0);
                conveyorBelt.setPower(0);
            }

            //LAUNCH SYSTEM
            if (gamepad1.left_bumper){
                launchRight.setPower(launchLow);
                launchLeft.setPower(launchLow);
            }
            else if (gamepad1.right_bumper){
                launchLeft.setPower(launchHigh);
                launchRight.setPower(launchHigh);
            }
            else{
                launchLeft.setPower(0);
                launchRight.setPower(0);
            }
            //TRIGGER
            if(gamepad1.x && isPressed == false)
            {
                isPressed = true;
                Trigger.setPosition(trigger_extended);
                sleep(500);
                Trigger.setPosition(trigger_retracted);
            }
            else
            {
                isPressed = false;
            }
            //WOBBLE GOAL
            if (gamepad2.x){
                wobbleGoal.activateClaw();
            }
            else if (gamepad2.b){
                wobbleGoal.setliftClawPosition();
            }
            else if (gamepad2.back){
                wobbleGoal.storeClaw();
            }
            if (gamepad2.y){
                wobbleGoal.raiseWobbleGoal();
            }
            else if (gamepad2.a){
                wobbleGoal.lowerWobbleGoal();
            }
            else {
                wobbleGoal.stopGoal();
            }
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), center (%.2f)", leftPower, rightPower, centerPower);
            telemetry.update();
        }
    }
}


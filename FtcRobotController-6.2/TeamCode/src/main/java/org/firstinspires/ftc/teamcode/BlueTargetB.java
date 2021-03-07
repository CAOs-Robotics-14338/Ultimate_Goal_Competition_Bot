package org.firstinspires.ftc.teamcode;

//Importing required classes
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.gyro;
import com.qualcomm.hardware.bosch.BNO055IMU;


/**
 *
 *
 * May add a variable or another class that will park the robot on the left or right half of the alliance bridge
 *
 *
 * */
// Declaring autonomous named Servo_Autonomous with the ground test
@Autonomous(name="A Mysterious Autonomous Appears", group="Blue")
// Creating class named servo autonomous that uses linear op mode
public class BlueTargetB extends LinearOpMode {

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

    /* GYRO */
    BNO055IMU               imu;
    Orientation lastAngles = new Orientation();
    gyro Gyro;


    /* WOBBLE */
    private DcMotor linearSlide = null;
    private Servo servo = null;
    private WobbleGoal wobbleGoal = null;

    // Starting OPMode
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


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        /* DRIVE */
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        centerDrive.setDirection(DcMotor.Direction.REVERSE);

        /* INTAKE */
        conveyorBelt.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(CRServo.Direction.FORWARD); //Setting up so positive is intaking but negative is pushing the rings away

        /* LAUNCHER */
        launchLeft.setDirection(DcMotor.Direction.REVERSE);
        launchRight.setDirection(DcMotor.Direction.FORWARD);

        /* WOBBLE GOAL */
        linearSlide  = hardwareMap.get(DcMotor.class, "slide");
        servo = hardwareMap.get(Servo.class, "claw");

        wobbleGoal = new WobbleGoal(linearSlide, servo);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();



        // Drive forward to the goal
        while (opModeIsActive() && runtime.seconds() < 2){
            leftDrive.setPower(0.30);
            rightDrive.setPower(0.30);
            // Adding telemetry data of our direction and run time
            telemetry.addData("Path", "TIME: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        Gyro.rotate(25,0.3);
        sleep(200);
        // Launch
        launchLeft.setPower(0.35);
        launchRight.setPower(0.35);
        sleep(1000);
        for(int i = 0; i < 3; i++)
        {
            Trigger.setPosition(trigger_extended);
            sleep(850);
            Trigger.setPosition(trigger_retracted);
            sleep(500);
        }
        launchLeft.setPower(0);
        launchRight.setPower(0);


        while (opModeIsActive() && runtime.seconds() < 2){
            leftDrive.setPower(0.30);
            rightDrive.setPower(0.30);
            // Adding telemetry data of our direction and run time
            telemetry.addData("Path", "TIME: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        sleep(200);
        // Drop wobble goal
        servo.setPosition(0.25);
        sleep(200);

        while (opModeIsActive() && runtime.seconds() < 3){
            leftDrive.setPower(-0.30);
            rightDrive.setPower(-0.30);
            // Adding telemetry data of our direction and run time
            telemetry.addData("Path", "TIME: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }


    }


}
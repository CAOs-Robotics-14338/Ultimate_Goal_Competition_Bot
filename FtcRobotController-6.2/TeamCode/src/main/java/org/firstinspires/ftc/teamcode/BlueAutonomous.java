package org.firstinspires.ftc.teamcode;

//Importing required classes
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.gyro;
import com.qualcomm.hardware.bosch.BNO055IMU;



// Declaring autonomous named Servo_Autonomous with the ground test
@Autonomous(name="A Mysterious Blue Autonomous Appears", group="Blue")
// Creating class named servo autonomous that uses linear op mode
public class BlueAutonomous extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    /* DRIVE */
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor centerDrive = null;
    HDrive hdrive = null;

    // Left 2847   Right -2845

    /* INTAKE */
    private DcMotor conveyorBelt = null;
    private CRServo intake = null;
    Intake intakeSystem = null;

    /* LAUNCHER */
    private DcMotorEx launchLeft = null;
    private DcMotorEx launchRight = null;

    /* TRIGGER */
    private Servo trigger = null;
    LaunchSystem launchSystem = null;




    /*WOBBLE GOAL*/
    private DcMotor linearSlide = null;
    private Servo servo = null;
    private WobbleGoal wobbleGoal = null;
    private double trigger_extended = 0.76;
    private double trigger_retracted = -0.86;
    private Boolean isPressed = false;

    /* GYRO */
    gyro Gyro;
    BNO055IMU imu;

    Orientation lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;


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
        hdrive = new HDrive(leftDrive, rightDrive, centerDrive, 'y');





        /* WOBBLE GOAL */
        linearSlide  = hardwareMap.get(DcMotor.class, "slide");
        servo = hardwareMap.get(Servo.class, "claw");

        wobbleGoal = new WobbleGoal(linearSlide, servo);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        /* INTAKE */
        conveyorBelt = hardwareMap.get(DcMotor.class, "belt");
        intake = hardwareMap.get(CRServo.class, "intake");
        intakeSystem = new Intake(conveyorBelt, intake);



        /* LAUNCHER */
        launchLeft = hardwareMap.get(DcMotorEx.class, "launch_left");;
        launchRight = hardwareMap.get(DcMotorEx.class, "launch_right");;
        launchRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launchLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        /* TRIGGER */
        trigger = hardwareMap.get(Servo.class, "trigger");
        launchSystem = new LaunchSystem(launchRight, launchLeft, trigger);

        Boolean isPressed = false;

        /* WOBBLE GOAL*/
        linearSlide  = hardwareMap.get(DcMotor.class, "slide");
        servo = hardwareMap.get(Servo.class, "claw");
        wobbleGoal = new WobbleGoal(linearSlide, servo);

        /* INTAKE */
        conveyorBelt.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(CRServo.Direction.FORWARD); //Setting up so positive is intaking but negative is pushing the rings away

        /* LAUNCHER */
        launchLeft.setDirection(DcMotorEx.Direction.REVERSE);
        launchRight.setDirection(DcMotorEx.Direction.FORWARD);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        /* DRIVE */




        /* IMU */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        Gyro = new gyro(leftDrive, rightDrive, imu);



        waitForStart();
        runtime.reset();


        leftDrive.setTargetPosition(2750);
        rightDrive.setTargetPosition(-2780);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(0.4);
        rightDrive.setPower(0.4);

        while(leftDrive.isBusy() || rightDrive.isBusy()){}



        // Launching 3 rings at low power into the high goal
        launchSystem.launchWheelsToLOWPower();
        sleep(2500);
        launchSystem.triggerLaunch();
        sleep(900);
        launchSystem.triggerBack();
        sleep(1500);
        launchSystem.triggerLaunch();
        sleep(900);
        launchSystem.triggerBack();
        sleep(1500);
        launchSystem.triggerLaunch();
        sleep(900);
        launchSystem.triggerBack();
        launchSystem.noLaunchWheels();

        launchRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launchLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setTargetPosition(200);
        rightDrive.setTargetPosition(-200);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(leftDrive.isBusy() || rightDrive.isBusy()){}

        leftDrive.setPower(0.4);
        rightDrive.setPower(0.4);
        wobbleGoal.deliverWobbleGoal();




/*


        // Drive forward to the launching position
        while (opModeIsActive() && runtime.seconds() < 3.5){
            leftDrive.setPower(0.30);
            rightDrive.setPower(0.30);
            // Adding telemetry data of our direction and run time
            telemetry.addData("Path", "TIME: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        runtime.reset();
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        Gyro.rotate(-12, 0.2);
        sleep(500);
        // Launch
        launchSystem.launchWheelsToLOWPower();
        sleep(1000);
        trigger.setPosition(trigger_extended);
        sleep(850);
        trigger.setPosition(trigger_retracted);
        sleep(500);
        sleep(1000);
        trigger.setPosition(trigger_extended);
        sleep(850);
        trigger.setPosition(trigger_retracted);
        sleep(500);
        sleep(1000);
        trigger.setPosition(trigger_extended);
        sleep(850);
        trigger.setPosition(trigger_retracted);
        sleep(500);
        launchSystem.noLaunchWheels();

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2.5){
            leftDrive.setPower(0.30);
            rightDrive.setPower(0.30);
            // Adding telemetry data of our direction and run time
            telemetry.addData("Path", "TIME: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        sleep(200);
        wobbleGoal.activateClaw();
        sleep(500);

        /*
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < 2){
            leftDrive.setPower(0.30);
            rightDrive.setPower(0.30);
            // Adding telemetry data of our direction and run time
            telemetry.addData("Path", "TIME: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        runtime.reset();

        sleep(200);
        // Drop wobble goal
        servo.setPosition(0.25);
        sleep(200);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 3){
            leftDrive.setPower(-0.30);
            rightDrive.setPower(-0.30);
            // Adding telemetry data of our direction and run time
            telemetry.addData("Path", "TIME: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

         */


    }


}
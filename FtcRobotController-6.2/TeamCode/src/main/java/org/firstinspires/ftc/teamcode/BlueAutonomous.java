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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.gyro;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

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

    /* Wobble Goal Target Variables */
    private int pos;
    private int position = 2;


    /* GYRO */
    gyro Gyro;
    BNO055IMU imu;

    Orientation lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;


    /* VISION */
    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;

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


        /* WOBBLE GOAL*/
        linearSlide  = hardwareMap.get(DcMotor.class, "slide");
        servo = hardwareMap.get(Servo.class, "claw");
        wobbleGoal = new WobbleGoal(linearSlide, servo);



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

        /* VISION */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        /* Updating telemetry with the open CV pipeline analysis and the assesed position */
        while(!isStarted()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.addData("Average", pipeline.avg1);
            telemetry.update();
        }
        //////WAITING FOR START BUTTON then restting the timer
        //waitForStart();
        runtime.reset();
        int ringPOS = pipeline.getAnalysis();
        /*
         Driving forward the measured distance using encoder algorithm with a slight turn to the left
         using more distance on the right wheel
        */
        hdrive.driveInches(60.62, 61.5, 0.4);


        /* Launching 3 rings at low power into the high goal
            Waiting a generous amount between each shot to ensure the motor stabilizes velocity
            using its PID loop
         */

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
        sleep(200);

        /* Updating telemetry with the open CV pipeline analysis and the assesed position */
        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();


        /* Using the OpenCV pipeline's assesment to determine what the starting stack and wobble goal target */


        if(ringPOS < pipeline.getONE_RING_THRESHOLD()) // No rings detected on the starter stack
        {
            /*
            We drive forward a measured distance, deliver the wobble goal to goal A,
             then reverse and park over the launch line for navigation points
             */
            hdrive.driveInches(16, 16, 0.4);
            sleep(400);
            wobbleGoal.activateClaw();
            sleep(400);
            hdrive.driveInches(-10, -10, -0.4);
            sleep(400);
        }
        else if(ringPOS > pipeline.getONE_RING_THRESHOLD() && ringPOS < pipeline.getFOUR_RING_THRESHOLD())
        { /* ONE RING */
            /*
            We drive forward a measured distance, deliver the wobble goal to goal B,
             then reverse and park over the launch line for navigation points
             */
            hdrive.driveInches(35, 8, 0.4);
            sleep(400);
            wobbleGoal.activateClaw();
            sleep(400);
            hdrive.driveInches(25, 4, 0.4);
            sleep(400);

        }
        else{ //FOUR RINGS
            /*
            We drive forward a measured distance, deliver the wobble goal to goal C,
             then reverse and park over the launch line for navigation points
             */
            hdrive.driveInches(32, 33.5, 0.4);
            sleep(400);
            wobbleGoal.activateClaw();
            sleep(400);
            hdrive.driveInches(-30, -30, -0.4);
            sleep(400);
        }


    }
    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(280,53); //x:181 y:98

        static final int REGION_WIDTH = 35; //35
        static final int REGION_HEIGHT = 25; //25

        final int FOUR_RING_THRESHOLD = 150; //prev 150
        final int ONE_RING_THRESHOLD = 135; //prev 135

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile Vision.SkystoneDeterminationPipeline.RingPosition position = Vision.SkystoneDeterminationPipeline.RingPosition.FOUR; //RingPosition.FOUR

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = Vision.SkystoneDeterminationPipeline.RingPosition.NONE; //.FOUR// Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = Vision.SkystoneDeterminationPipeline.RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = Vision.SkystoneDeterminationPipeline.RingPosition.ONE;
            }else{
                position = Vision.SkystoneDeterminationPipeline.RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
        public int getFOUR_RING_THRESHOLD(){
            return FOUR_RING_THRESHOLD;
        }

        public int getONE_RING_THRESHOLD() {
            return ONE_RING_THRESHOLD;
        }
    }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class HDrive {

    /* Drive Motors */
    private DcMotor leftDrive, rightDrive, centerDrive;
    double ticksPerRev = 538.6; // ticks per revolution
    double circumference = 11.874; //inch circumference
    double ticksPerInch = ticksPerRev/circumference; //45.3596
    int ticksToDestinationLeft, ticksToDestinationRight;

    //Constructor
    public HDrive(DcMotor LeftDriveMotor, DcMotor RightDriveMotor, DcMotor CenterDriveMotor, char autonomous)
    {
        this.leftDrive = LeftDriveMotor;
        this.rightDrive = RightDriveMotor;
        this.centerDrive = CenterDriveMotor;

        if(autonomous != 'y' || autonomous != 'Y')
        {
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            /* DRIVE */
            leftDrive.setDirection(DcMotor.Direction.REVERSE);
            rightDrive.setDirection(DcMotor.Direction.FORWARD);
            centerDrive.setDirection(DcMotor.Direction.REVERSE);

            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            centerDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else{ //TELEOP Drive Motor Settings

            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            /* DRIVE */
            leftDrive.setDirection(DcMotor.Direction.REVERSE);
            rightDrive.setDirection(DcMotor.Direction.REVERSE);
            centerDrive.setDirection(DcMotor.Direction.REVERSE);

            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            centerDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void driveInches(double distanceLeft, double distanceRight, double power)
    {
        this.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ticksToDestinationLeft = (int)(ticksPerInch * distanceLeft);
        ticksToDestinationRight = (int)(ticksPerInch * distanceRight);


        this.leftDrive.setTargetPosition(ticksToDestinationLeft);
        this.rightDrive.setTargetPosition(ticksToDestinationRight);

        this.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        this.rightDrive.setPower(power);
        this.leftDrive.setPower(power);

        while(this.rightDrive.isBusy() || this.leftDrive.isBusy()){}
    }

}
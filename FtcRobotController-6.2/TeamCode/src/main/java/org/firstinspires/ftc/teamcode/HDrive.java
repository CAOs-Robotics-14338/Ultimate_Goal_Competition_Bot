package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class HDrive {

    /* Drive Motors */
    private DcMotor leftDrive, rightDrive, centerDrive;


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
            rightDrive.setDirection(DcMotor.Direction.REVERSE);
            centerDrive.setDirection(DcMotor.Direction.REVERSE);

            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            centerDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else{ //TELEOP Drive Motor Settings

        }
    }

}
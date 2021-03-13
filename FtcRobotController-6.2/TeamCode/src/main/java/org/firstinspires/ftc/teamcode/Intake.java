package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {
    // Intake wheels and the conveyor belt
    DcMotor conveyor;
    CRServo intake;

    double conveyorPower = 1;
    double intakePower = 1;


    public Intake(DcMotor conveyorBelt, CRServo intakeWheels){
        this.conveyor = conveyorBelt;
        this.intake = intakeWheels;

        conveyor.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(CRServo.Direction.FORWARD); //Setting up so positive is intaking but negative is pushing the rings away
    }

    public void intake(){
        intake.setPower(intakePower);
        conveyor.setPower(conveyorPower);
    }

    public void outtake (){
        intake.setPower(-intakePower);
        conveyor.setPower(-conveyorPower);
    }
    public void stop(){
        intake.setPower(0);
        conveyor.setPower(0);
    }
}

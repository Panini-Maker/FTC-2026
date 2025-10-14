/*
Please note that at the current moment, this class is not used and is left here to be used in the future

Reasons for this:
 * There waa an error when the movement code was placed in here.
 */

//This defines the package the class rests in
package org.firstinspires.ftc.teamcode.lib;

//These import statements import the necessary items that will be used in the code
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//This declares the class
public class TwoMotorDrive {
    //This declares four variables, one for each motor, their names will explain
    //what their function is
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public DcMotor leftExtension;
    public DcMotor rightExtension;
    public ElapsedTime runtime;

    //This method sets the motors in this class to the motor in the class "TestTeleOp"
    public void init(DcMotor leftMotor, DcMotor rightMotor,
                     DcMotor rightExtension, DcMotor leftExtension,
                     ElapsedTime runtime) {
        //The "this.[...]" sets the variables equal to the input values
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.rightExtension = rightExtension;
        this.leftExtension = leftExtension;
        this.runtime = runtime;
    }

    //This method sets the motor speed, but is not used at the
    //current moment due to some issues
    public void setPowers(double power, double right) {
        leftMotor.setPower(power + right);
        rightMotor.setPower(power - right);
    }

    //This method sets the motor speed, but is not used at the
    //current moment due to some issues
    public void setArmPower(double extensionPower) {
        leftMotor.setPower(extensionPower);
        rightMotor.setPower(extensionPower);
    }
}

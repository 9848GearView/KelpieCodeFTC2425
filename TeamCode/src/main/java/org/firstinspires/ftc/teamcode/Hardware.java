package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.technototes.library.hardware.HardwareDevice;
import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.hardware.motor.EncodedMotorGroup;
import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.hardware.motor.MotorGroup;
import org.firstinspires.ftc.teamcode.subsystems.GyroSensor;
import com.technototes.library.hardware.sensor.IMU;
import com.technototes.library.hardware.servo.Servo;
import com.technototes.logger.Loggable;

/** Class for the hardware devices of the robot
 *
 */
public class Hardware implements Loggable {

    //drivebase
    public EncodedMotor<DcMotor> flDriveMotor;
    public EncodedMotor<DcMotor> frDriveMotor;
    public EncodedMotor<DcMotor> rlDriveMotor;
    public EncodedMotor<DcMotor> rrDriveMotor;

    public GyroSensor imu;

    //index
    public Servo indexArmServo;
    public Servo indexPivotServo;

    //intake
    public Motor<DcMotor> intakeMotor1;
    public Motor<DcMotor> intakeMotor2;
    public MotorGroup intakeMotorGroup;

    //shooter
    public EncodedMotor<DcMotor> shooterMotor1;
    public EncodedMotor<DcMotor> shooterMotor2;
    public EncodedMotorGroup shooterMotorGroup;


    //wobble
    public Servo wobbleArmServo;
    public Servo wobbleClawServo;

    public Hardware(){
        flDriveMotor = new EncodedMotor<>("flMotor");
        frDriveMotor = new EncodedMotor<>("frMotor");
        rlDriveMotor = new EncodedMotor<>("rlMotor");
        rrDriveMotor = new EncodedMotor<>("rrMotor");

        //TODO fix
        imu = new GyroSensor("imu");

        indexArmServo = new Servo("indexarm");
        indexPivotServo = new Servo("indexpivot");

        intakeMotor1 = new Motor<>("intake1");
        intakeMotor2 = new Motor<>("intake2");
        intakeMotorGroup = new MotorGroup(intakeMotor1, intakeMotor2);

        shooterMotor1 = new EncodedMotor<>("shooter1");
        shooterMotor2 = new EncodedMotor<>("shooter2");
        shooterMotorGroup = new EncodedMotorGroup(shooterMotor1.invert(), shooterMotor2.invert());

        wobbleArmServo = new Servo("wobblearm");
        wobbleClawServo = new Servo("wobbleclaw");
    }
}

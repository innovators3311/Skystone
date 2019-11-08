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

package org.firstinspires.ftc.teamcode.Skystone.ChickenLittle;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Set;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbot
{
    /* Public OpMode members. */
    private DcMotor  rightFrontMotor   = null;
    private DcMotor  leftFrontMotor  = null;
    private DcMotor  rightBackMotor     = null;
    private DcMotor  leftBackMotor   =null;
    Thread runoooooo;
    double rf_target;
    double rb_target;
    double lf_target;
    double lb_target;
    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    boolean userubberbanding = true;
    LinearOpMode parent;
    final double MTP = 2;
    double MinSleepPeriod = .01;
    double SPM = .1;//MTP/MinSleepPeriod;
    double globalAngle;
    Orientation             lastAngles = new Orientation();
    BNO055IMU imu;




    public boolean setMotors(double Left, double Right)
    {
        if(leftFrontMotor==null ||rightFrontMotor==null )
            return false;
        leftFrontMotor.setPower(Left);
        leftBackMotor.setPower(Left);

        rightFrontMotor.setPower(Right);
        rightBackMotor.setPower(Right);
        return true;
    }
    /* Constructor */
    public HardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode parentIN) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        parent = parentIN;
        // Define and Initialize Motors
        rightFrontMotor  = hwMap.get(DcMotor.class, "rf");
        leftFrontMotor = hwMap.get(DcMotor.class, "lf");
        rightBackMotor    = hwMap.get(DcMotor.class, "rb");
        leftBackMotor   = hwMap.get (DcMotor.class, "lb");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //Vertical brain
        //            int AXIS_MAP_CONFIG_BYTE = 0x18;
//            imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
//            Thread.sleep(100);
//            imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE & 0x0F);
//            imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_CONFIG_BYTE & 0x0F);
//            imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);
//            Thread.sleep(100);
        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        new Thread(new rubberbandRunnable(parent)).start();

    }


    class rubberbandRunnable  implements Runnable {


        LinearOpMode parent2;

        boolean keepRunning=true;
        public rubberbandRunnable(LinearOpMode parentIN) {
            parent2 = parentIN;

        }


        private void rubberMotor(DcMotor motor, double target)
        {
            double difference= target-motor.getPower();
            //
            if (difference<0)
            {
                if (difference<-1*SPM)
                {
                    float NewMotorSettings= (float)(motor.getPower()-SPM);
                    motor.setPower(NewMotorSettings);
                }
                else
                {
                    motor.setPower(target);
                }
            }
            else
            {
                if (difference>SPM)
                {
                    float NewMotorSettings= (float)(motor.getPower()+SPM);
                    motor.setPower(NewMotorSettings);
                }
                else
                {
                    motor.setPower(target);
                }
            }
        }

        @Override
        public void run() {
            int counter = 0;


            parent.waitForStart();
            parent.telemetry.addData("Thread","about to start");
            parent.telemetry.update();

            while (keepRunning)
            {
                parent.telemetry.addData("Thread","running");
                parent.telemetry.update();
                rubberMotor(leftFrontMotor, lf_target);
                rubberMotor(rightFrontMotor, rf_target);
                rubberMotor(leftBackMotor,lb_target);
                rubberMotor(rightBackMotor, rb_target);

                try {
                    Thread.sleep((int)(1000*MinSleepPeriod));
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }


//                if (parent!=null)
//                {
//                    parent.telemetry.addData("","c i "+rf_target);
//
//                    parent.telemetry.update();
//                }

            }

            parent.telemetry.addData("Thread","about to die");
            parent.telemetry.update();
        }

   }


    public void setMotors (double lf, double lb, double rf, double rb){
        if (userubberbanding= true)
        {
            rf_target = rf;
            rb_target = rb;
            lf_target = lf;
            lb_target = lb;
        }
        else
        {
            rightFrontMotor.setPower(rf);
            leftFrontMotor.setPower(lf);
            rightBackMotor.setPower(rb);
            leftBackMotor.setPower(lb);
        }
    }

    public void FlowAngleStrafeTime( boolean StrafeLeft, float speed, float time_s, LinearOpMode opmode)
    {
        double compass_ideal_heading=-1;
        ElapsedTime     runtime = new ElapsedTime();
        runtime.reset();
        compass_ideal_heading=getAngle();
        while (opmode.opModeIsActive() && (runtime.seconds() < time_s)) {

            if(StrafeLeft) {
                double angle_difference = (compass_ideal_heading - getAngle()) / 200.0;
                leftBackMotor.setPower(.5 - angle_difference);
                leftFrontMotor.setPower(-.5 - angle_difference);
                rightBackMotor.setPower(-.5 + angle_difference);
                rightFrontMotor.setPower(.5 + angle_difference);
            }

            else{
                double angle_difference=(compass_ideal_heading-getAngle())/200.0;
                leftBackMotor.setPower(-.5-angle_difference);
                leftFrontMotor.setPower(.5-angle_difference);
                rightBackMotor.setPower(.5+angle_difference);
                rightFrontMotor.setPower(-.5+angle_difference);
            }
            opmode.idle();
        }
        setMotors(0,0);
    }
    public double getAngle()
    {
        try{
            // We experimentally determined the Z axis is the axis we want to use for heading angle.
            // We have to process the angle because the imu works in euler angles so the Z axis is
            // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
            // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

            if (deltaAngle < -180)
                deltaAngle += 360;
            else if (deltaAngle > 180)
                deltaAngle -= 360;

            globalAngle += deltaAngle;

            lastAngles = angles;

            return globalAngle;}

        catch(Exception e){
            return 0;
        }
    }

    public void TurnToAngle(float targetangle, float speed) {

    }
    public void Drive(float distance, float angle, float speed){

    }
    public void Extend(float length, float speed, float time){

    }
    public void StrafeToAngle(float distance, float angle){

    }
 }


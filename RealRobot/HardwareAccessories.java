package org.firstinspires.ftc.teamcode.Skystone.RealRobot;

import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.List;


public class HardwareAccessories {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    public TFObjectDetector tfod;
    public DcMotor ArmUpDown   = null, ArmInOut=null;
    public TouchSensor ArmDowm;
    public TouchSensor ArmUp;
    public LinearOpMode OpMode;
    HardwareMap hwMap           =  null;
    File captureDirectory = AppUtil.ROBOT_DATA_DIR;

    private static final String VUFORIA_KEY = "AXTF/LP/////AAABmS/x33JhJkAynrU/ggLtC+5u5UI4i6vN/nZ7SlkLSyJY0DibuV6b/MqkjFB3Bg8y1BxP7ODkNV31aw64HP19MxyVGC4mMf7bTAnb7ORwgRebgahyUezUi+z3LCQO0FrciWVbFG8MrlupKvtZllPW3pH5pO0OJljn7kA1jjuKpQhikG6+TUOIAejFIPHLxGUyBk6i+w3MhxN6hFbX5NMjHbFIV4fbTc0yC8CUK6mSXdlrMt4dE89taWBOSaP5UOXNTbws5N5+84U4eJFjFi1AFsNJOUDYC4XQOoIjH9lb9pCy8p7egNJXJoOPeooW7cfpqRcfLnOetqNpdvgTadNE2ARLaL6pGBR6iXq6FM0sYzvV";
    private VuforiaLocalizer vuforia;
    WebcamName webcamName;

    public void init(HardwareMap ahwMap, LinearOpMode parent,boolean initalizeTfod)
    {
        hwMap = ahwMap;
        OpMode = parent;
/////////////////arm set up/////////////////////////////////////////////////////////////////////////////////////////////////
        ArmInOut = hwMap.get(DcMotor.class, "arm_in_out");
        ArmUpDown = hwMap.get(DcMotor.class, "arm_up_down");
        ArmInOut.setPower(0);
        ArmUpDown.setPower(0);
        ArmUpDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmUpDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmInOut.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmUpDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmInOut.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ArmDowm = hwMap.touchSensor.get("arm_down");
        ArmUp = hwMap.touchSensor.get("arm_up");
        if(initalizeTfod)
        {
            try {
                initVuforia();
            }
            catch(Exception e) {
                parent.telemetry.addData("FindBlock", "Failed in init"+e);
                parent.telemetry.update();

            }


            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
            } else {
                parent.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }

            if (tfod != null) {
                tfod.activate();
            }
        }

    }


    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public Direction FindSkystone() {

        Direction result=Direction.LEFT;

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            for(int i=0;i<25;i++)
            {
                if (updatedRecognitions != null) {
                    OpMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.


                    int skystoneleft = 600;
                    for (Recognition recognition : updatedRecognitions) {

                        if (recognition.getLabel().contains("Sky")) {
                            skystoneleft = (int) recognition.getLeft();
                        }

                        //telemetry.addData(String.format("label (%d) ",  i), String.format("%s",recognition.getLabel()));
                        //telemetry.addData(String.format("  left,top (%d)", i), "%.0f , %.0f %.1f   %.1f",
                        //recognition.getLeft(), recognition.getTop(), recognition.estimateAngleToObject(AngleUnit.DEGREES), recognition.getConfidence());
                    }
                    if (skystoneleft < 150) {
                        result = Direction.LEFT;

                        OpMode.telemetry.addData(String.format("answer"), String.format("left") + skystoneleft);
                    } else if (skystoneleft < 400) {

                        result = Direction.CENTER;
                        OpMode.telemetry.addData(String.format("answer"), String.format("middle") + skystoneleft);
                    } else {

                        result = Direction.RIGHT;
                        OpMode.telemetry.addData(String.format("answer"), String.format("right") + skystoneleft);
                    }

                    OpMode.telemetry.update();
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        }
        return result;
    }

}

package org.firstinspires.ftc.teamcode.Concept;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.hardware.limelightvision.LLResultTypes.CalibrationResult;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/*
 * VV LimeLight3A class
 * Pipeline 1 is our Neural Detector with "red", "blue", and "yellow"
 * Neural Detectors are identifiable until 1.5 ft */

@TeleOp(name = "NeuralDetectorTest", group = "Sensor")

public class vvLimeLightNeural extends LinearOpMode {

    private Limelight3A limelight;

    public boolean initLL()
    {
        boolean isInit = false;
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            if(limelight != null) {
                telemetry.setMsTransmissionInterval(11); //why 11
                // Switch to the neural detector pipeline
                limelight.pipelineSwitch(1);
                if(limelight.isConnected())
                    isInit = true;
            }
        } catch (Exception e) {
            telemetry.addData(">", "Error in InitLimeLight." + e);
        }
        return isInit;
    }

    public boolean StopLimelight()
    {
        boolean isStop = false;
        try
        {
            if(limelight != null)
            {
                if(limelight.isRunning()) {
                    limelight.stop();
                    if (limelight.isRunning())
                        isStop = true;
                }
                else
                    isStop = true;
            }
        } catch (Exception e) {
            telemetry.addData(">", "Error in StopLimeLight." + e);
        }
        return isStop;
    }

    public boolean startLimelight()
    {
        boolean isStart = false;
        try {
            if(limelight != null) {
                limelight.start();
                telemetry.addData(">", "Robot Ready for Neural Detector. Press Play.");
                telemetry.update();
                waitForStart(); //what this function will do
                if(limelight.isRunning())
                    isStart = true;
            }
        } catch (Exception e) {
            telemetry.addData(">", "Error in Starting LimeLight." + e);
        }
        return isStart;
    }

    public void getLimeLightStatus()
    {
        if (limelight != null) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s", status.getName());
            //Hardware metric information
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d , RAM: %.1f",
                    status.getTemp(), status.getCpu(), (int) status.getFps(), status.getRam());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());
        }
    }

    public String getDetectorName()
    {
        String detectorName ="";
        if(limelight != null)
        {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid() )
            {
                // Access detector results for neural detection
                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                if(detectorResults != null && !detectorResults.isEmpty()) {
                    for (LLResultTypes.DetectorResult dr : detectorResults)
                    {
                        detectorName = dr.getClassName();
                        //telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                    }
                }
            }
        }
        return detectorName;
    }

    public void getLatestResult()
    {
        if(limelight != null)
        {
            LLResult result = limelight.getLatestResult();
            if (result != null)
            {
                // Access general information
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);

                if (result.isValid())
                {
                    telemetry.addData("tx", result.getTx());
                    // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
                    telemetry.addData("ty", result.getTy());
                    //Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees)
                    telemetry.addData("tync", result.getTyNC()); //Horizontal Offset From Principal Pixel To Target (degrees)
                    telemetry.addData("txnc", result.getTxNC()); //Horizontal Offset From Principal Pixel To Target (degrees)
                    telemetry.addData("Botpose", botpose.toString());

                    // Access classifier results
                    List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                    telemetry.addData("Classifier Count", classifierResults.size());
                    for (LLResultTypes.ClassifierResult cr : classifierResults) {
                        telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                    }

                    // Access detector results for neural detection
                    List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                    for (LLResultTypes.DetectorResult dr : detectorResults)
                    {
                        telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                    }
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }
        }
        telemetry.update();
    }

    /*
    @Override

    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        // Switch to the neural detector pipeline
        limelight.pipelineSwitch(1); // We use pipleline 1 as we need to train model to identify the object and color and for pipeline 2 this version of limelight don't have support to detect color

        // Starts polling for data
        limelight.start();

        telemetry.addData(">", "Robot Ready for Neural Detector. Press Play.");
        telemetry.update();
        waitForStart();
        while (opModeIsActive())
        {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s", status.getName());
            //Hardware metric information
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d , RAM: %.1f",
                    status.getTemp(), status.getCpu(), (int) status.getFps(),status.getRam());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                // Access general information
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);

                if (result.isValid()) {
                    telemetry.addData("tx", result.getTx());
                    // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
                    telemetry.addData("ty", result.getTy());
                    //Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees)
                    telemetry.addData("tync", result.getTyNC()); //Horizontal Offset From Principal Pixel To Target (degrees)
                    telemetry.addData("txnc", result.getTxNC()); //Horizontal Offset From Principal Pixel To Target (degrees)


                    telemetry.addData("Botpose", botpose.toString());

                    // Access classifier results
                    List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                    telemetry.addData("Classifier Count", classifierResults.size());
                    for (LLResultTypes.ClassifierResult cr : classifierResults) {
                        telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                    }

                    // Access detector results for neural detection
                    List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                    for (LLResultTypes.DetectorResult dr : detectorResults) {
                        telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                    }
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }

            telemetry.update();
        }
        limelight.stop();
    }//en of ru nop mode

    */


    @Override

    public void runOpMode() throws InterruptedException
    {
        boolean isSuccess = initLL();
        if(isSuccess)
            isSuccess = startLimelight();
        if(isSuccess) {
            while (opModeIsActive()) {
                getLimeLightStatus();
                getLatestResult();
            }
            isSuccess = StopLimelight();
            if (!isSuccess) {
                telemetry.addData("Limelight", "No data available");
            }
        }

    }//en of ru nop mode

}
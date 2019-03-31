package org.firstinspires.ftc.teamcode.Network;

import android.graphics.Bitmap;
import android.util.Log;
import org.opencv.android.Utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.tensorflow.lite.Interpreter;

import java.io.File;
import java.util.HashMap;
import java.util.Map;

public class Model {
    private static final String VUFORIA_KEY = "AVtZQ6n/////AAABmdcWQU7kykWgmYCE0DI/QOReCtZljUv/ks9BxJvDlzzFaMhm4I4BBhWA8BDMwM6wDclf7C3Uejvm+pnib7YV+D/n8iAQdR7MAGlBGRqXbUPG1HwHfKCK27WTAuNxHilwwMcEIyPjgJY+9ozAZtnbaWzCDZjrNC1WlClxqnGMT5qO93K2ARRy+3FKtNV93opS7YAVfhRSNxWh/bBRa05OWKjB41PdctoT1IWWsabSad2Fvj7qzasRnG+cmO2ePVMxIwEPC2w1K6gQSFBsk97Sku2EmqgsnRYNzqnXpPn/tXhsQhmiTpWkZUnZTfTsnx3jxB6vdfoZA+JKBqrlfqqqxiBWLx72h0f31ek6TuwdwTTd";

    private VuforiaLocalizer vuforia;

    private HardwareMap hwMap;

    private Telemetry telemetry;

    private Action[] actions;

    private Interpreter model;

    //add model stuff
    public Model(HardwareMap hwMap, Telemetry telemetry, String modelName, Action[] actions){
        this.telemetry = telemetry;

        this.hwMap = hwMap;

        this.actions = actions;

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        String baseDirectory = "/sdcard/FIRST/CS/";
        String path = baseDirectory + modelName + ".tflite";
        File f = new File(path);
        model = new Interpreter(f);

    }

    public enum Action{
        Left,Right,Forwards,Backwards,CC,CCW
    }

    public Action predict(){
        Mat mat = processFrame(getFrame());
        int numActions = actions.length;
        double[] values = new double[numActions];
        for (int i = 0; i < numActions; i++){
            double[] actionInput = new double[numActions];
            actionInput[i] = 1;
            Object[] inputs = {mat,actionInput};
            double[] output = new double[1];
            Map<Integer,Object> outputs = new HashMap();
            outputs.put(0,output);
            model.runForMultipleInputsOutputs(inputs,outputs);
            values[i] = ((double) outputs.get(0));
        }
        int action = -1;
        double value = 0;
        for (int i = 0; i < numActions; i++){
           if(action == -1 || values[i] > value){
                action = i;
                value = values[i];
            }
        }
        Action actionName = actions[action];
        return actionName;
    }

    public Mat processFrame(Mat frame){
        return frame;
    }

    public Mat getFrame() {
        VuforiaLocalizer.CloseableFrame frame;
        Image rgb = null;

        try {
            // grab the last frame pushed onto the queue
            frame = vuforia.getFrameQueue().take();
        } catch (InterruptedException e) {
            Log.d("model", "Problem taking frame off Vuforia queue");
            e.printStackTrace();
            return null;
        }

        // basically get the number of formats for this frame
        long numImages = frame.getNumImages();

        // set rgb object if one of the formats is RGB565
        for(int i = 0; i < numImages; i++) {
            if(frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }

        if(rgb == null) {
            Log.d("model", "Image format not found");
            return null;
        }

        // create a new bitmap and copy the byte buffer returned by rgb.getPixels() to it
        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

        // construct an OpenCV mat from the bitmap using Utils.bitmapToMat()
        Mat mat = new Mat(bm.getWidth(), bm.getHeight(), CvType.CV_8UC4);
        Utils.bitmapToMat(bm, mat);

        // convert to BGR before returning
        //Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2BGR);

        frame.close();

        //Log.d(LOG_TAG, "Frame closed");

        return mat;
    }
}

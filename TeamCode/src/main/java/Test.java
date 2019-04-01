import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.tensorflow.lite.Interpreter;
import org.firstinspires.ftc.teamcode.Network.Model;
import org.firstinspires.ftc.teamcode.Network.Model.Action;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.File;

public class Test {
    public static void main(String[] args){
        //System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        //System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        //Action[] actions = {Action.Forwards,Action.CC,Action.CCW};
        //Model model = new Model("cnnS",actions);
        //Load an image
        String imageName = "30x60b";
        String directory = "./images/";
        File file = new File(directory + imageName +".png");
        Mat image = Imgcodecs.imread("test");
        System.out.println("width: " + image.width()+ "height: " + image.height()+"channels: " + image.channels());


    }
}

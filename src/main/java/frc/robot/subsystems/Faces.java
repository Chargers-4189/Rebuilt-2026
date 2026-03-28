package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

//import edu.wpi.first.wpilibj.util.Color;
import java.io.FileReader;
//import java.io.IOException;
//import java.util.HashMap;
//import java.util.Map;
import java.util.Properties;
//import edu.wpi.first.wpilibj.AddressableLED;
//import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

//import frc.robot.util.Elastic.ElasticFace;

public class Faces extends SubsystemBase {
    //private static final int LED_PIN = 1;//port number
    private static final int NUM_LEDS = 512;//number of lights (16 by 16 plus 8)
    //private static final double BRIGHTNESS = 0.5;

    private Timer time = new Timer();
    //private AddressableLED led = new AddressableLED(LED_PIN);
    //private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(NUM_LEDS);
    private CANdle candle = new CANdle(1);

    //private final double dampenFactor = 3;
    public String animation = "none";
    public int mode = 0;
    public double startTime = 0;

    //private static final Map<String, short[]> Face = new HashMap<String, short[]>();

    public short[] chosenFace = {  
        50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0
    };
    public short[]previousFace = chosenFace;
    public short[]defaultFace = chosenFace;
    //private final LEDPattern blue = LEDPattern.solid(new Color(.2, .2, .2));
    public Faces() {
        //led = new AddressableLED(LED_PIN);
        //led.setLength(NUM_LEDS);
        //led.start();
        //blue.applyTo(ledBuffer);
    }

    public void resetTime(){
       time.reset();
    }
    public void startTime(){
        time.start();
     }
    public double getTime(){
        return time.get();
    }

    public void clear() {
        candle.setControl(new SolidColor(0, (NUM_LEDS - 2)).withColor( new RGBWColor(0, 0, 0, 0)));
    }

    public void makeFace(short[] pixelsArray){
        for (int i = 0, j = 8; j < NUM_LEDS && i + 2 < pixelsArray.length; i += 3, j++) {  
            candle.setControl(new SolidColor(j, j).withColor( new RGBWColor((int) (pixelsArray[i]), (int) (pixelsArray[i + 1]), (int) (pixelsArray[i + 2]), 0)));
        }
        //led.setData(ledBuffer);
    }

    public void setFace(short[] face, String animationState){
        chosenFace = face;
        animation = animationState;
        if(animationState != "none"){
            startTime = Timer.getTimestamp();
        }
    }

    public short[] getFace(String name) {
        try {
            FileReader faceFile = new FileReader(Filesystem.getDeployDirectory() + "/faces.properties");
            Properties p = new Properties();
            p.load(faceFile);
            return convertToArray(p.getProperty(name));
        } catch (Exception e) {
            System.err.println("Error loading face properties: " + e);
            return defaultFace;
        }
    }

    public short[] convertToArray(String input) {
        String[] stringArray = input.split(",");
        short[] shortArray = new short[stringArray.length];
        for (int i = 0; i < stringArray.length; i++) {
            try {
                shortArray[i] = Short.parseShort(stringArray[i].trim());
            } catch (NumberFormatException e) {
                System.err.println("Error parsing short: " + e);
                return new short[0];
            }
        }
        return shortArray;
    }


    public void smile(){
        setFace(getFace("smile"), "none");
    }
    
    public void nextDizzy(int mode){
        if(mode == 0){
            setFace(getFace("dizzyOne"), "dizzy");
        }else if(mode == 1){
            setFace(getFace("dizzyTwo"), "dizzy");
        }else if(mode == 2){
            setFace(getFace("dizzyThree"), "dizzy");
        }else if(mode == 3){
            setFace(getFace("dizzyFour"), "dizzy");
        }
    }
    /** 
    public void submerge(int mode){
        if(mode == 0){
            setFace(getFace("waterOne"));
        }else if(mode == 1){
            setFace(getFace("waterTwo"));
        }else if(mode == 2){
            setFace(getFace("waterThree"));
        }else if(mode == 3){
            setFace(getFace("waterFour"));
        }else if(mode == 4){
            setFace(getFace("waterFive"));
        }else if(mode == 5){
            setFace(getFace("waterSix"));
        }else if(mode == 6){
            setFace(getFace("waterSeven"));
        }else if(mode == 7){
            setFace(getFace("waterEight"));
        }else if(mode == 8){
            setFace(getFace("waterNine"));
        }else if(mode == 9){
            setFace(getFace("waterTen"));
        }else if(mode == 10){
            setFace(getFace("waterEleven"));
        }else if(mode == 11){
            setFace(getFace("waterTwelve"));
        }else if(mode == 12){
            setFace(getFace("waterThirteen"));
        }else if(mode == 13){
            setFace(getFace("waterFourteen"));
        }else if(mode == 14){
            setFace(getFace("waterFifteen"));
        }else if(mode == 15){
            setFace(getFace("waterSixteen"));
        }
        makeFace(chosenFace);
    }*/
    public void jeremy(int mode){
        if(mode == 0){
            setFace(getFace("jeremyOne"), "jeremy");
        }else if(mode == 1){
            setFace(getFace("jeremyTwo"), "jeremy");
        }else if(mode == 2){
            setFace(getFace("jeremyThree"), "jeremy");
        }else if(mode == 3){
            setFace(getFace("jeremyTwo"), "jeremy");
        }
    }

    public void angry(int mode){
        if(mode == 0){
            setFace(getFace("smile"), "angry");
        }else if(mode == 1){
            setFace(getFace("angryTwo"), "angry");
        }else if(mode == 2){
            setFace(getFace("angryThree"), "angry");
        }else if(mode == 3){
            setFace(getFace("angryFour"), "angry");
        }else if(mode == 4){
            setFace(getFace("dead"), "angry");
        }
    }
    public void money(int mode){
        if(mode == 0){
            setFace(getFace("moneyOne"), "money");
        }else if(mode == 1){
            setFace(getFace("moneyTwo"), "money");
        }
    }
    public void party(int mode){
        if(mode == 0){
            setFace(getFace("partyOne"), "party");
        }else if(mode == 1){
            setFace(getFace("partyTwo"), "party");
        }
    }
    public void sleepy(){
        setFace(getFace("sleepy"), "none");
    }
    public void frown(){
        setFace(getFace("frown"), "none");
    }
    public void crying(){
        setFace(getFace("crying"), "none");
    }
    public void wink(){
        setFace(getFace("wink"), "none");
    }/* 
    public void speed(){
        setFace(getFace("speed"), "none");
        makeFace(chosenFace);
    }*/
   //I'm not gonna comment sick ans heart eyes out becuase thay seem pretty likeley to be needed
    public void sick(){
        setFace(getFace("sick"), "none");
    }
    public void heartEyes(){
        setFace(getFace("heartEyes"), "none");
    }
    public void fearShock(){
        setFace(getFace("fearShock"), "none");
    }/*
    public void eyebrowRaise(){
        setFace(getFace("eyebrowRaise"), "none");
        makeFace(chosenFace);
    } */
    public void pirate(){
        setFace(getFace("pirate"), "none");
    }/* 
    public void red(){
        System.out.println("Hello");
        setFace(getFace("red"));
        makeFace(chosenFace);
    }
    public void blue(){
        setFace(getFace("blue"));
        makeFace(chosenFace);
    }
    public void cupcake(){
        setFace(getFace("cupcake"));
        makeFace(chosenFace);
    }
    public void clown(){
        setFace(getFace("clown"));
        makeFace(chosenFace);
    }
    public void bad(){
        setFace(getFace("exclamationPoint"));
        makeFace(chosenFace);
    }*/

    @Override
    public void periodic() {
        if(animation != "none" && (Timer.getTimestamp() - startTime) >= 1){
            mode++;
            if(animation == "smile"){
                //smile(mode % 11);
            } 
            else if(animation == "jeremy"){
                jeremy(mode % 4);
            }
            else if(animation == "dizzy"){
                nextDizzy(mode % 5);
            }
            else if(animation == "party"){
                party(mode % 2);
            }
            else if(animation == "money"){
                money(mode % 2);
            }
            startTime = Timer.getTimestamp();
        }
        //led.setData(ledBuffer);
        //led.setData(ledBuffer);
        /*try {
        getFace("smile");
        } catch (Exception e) {
        // TODO: handle exception
        System.err.println(e);
        }*/
        makeFace(chosenFace);

    }
}

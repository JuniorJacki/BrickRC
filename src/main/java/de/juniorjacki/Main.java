package de.juniorjacki;

import de.juniorjacki.remotebrick.DataSubscriber;
import de.juniorjacki.remotebrick.Hub;
import de.juniorjacki.remotebrick.devices.ConnectedDevice;
import de.juniorjacki.remotebrick.devices.Motor;
import de.juniorjacki.remotebrick.devices.UltrasonicSensor;
import de.juniorjacki.remotebrick.types.Image;
import de.juniorjacki.remotebrick.types.PathDirection;
import de.juniorjacki.remotebrick.types.Port;
import de.juniorjacki.remotebrick.types.StopType;
import de.juniorjacki.utils.Logger;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class Main {
    public static void main(String[] args) {


        CompletableFuture.runAsync(() -> new Main("A8:E2:C1:9B:9B:C3","A8:E2:C1:9C:52:E1"));
        new Main("A8:E2:C1:9C:91:02","A8:E2:C1:9B:A8:F3");
    }


    public Main(String controllerMac,String robotMac) {
        Hub robot = null;
        Hub controller = null;
        int i = 0;
        while (robot == null || controller == null) {
            if(robot == null) robot = Hub.connect(robotMac);
            if(controller == null) controller = Hub.connect(controllerMac);
            Logger.info("Connection attempt " + i);
            try {
                Thread.sleep(100);
            } catch (InterruptedException ignored) {}
        }
        Logger.info("Connected Successfully");

        startNormalization(controller);
        startPowerControl(controller, robot);
        startTurnControl(controller, robot);
        startDistanceReport(controller, robot);
        startGrabControl(controller,robot);
        startLightControl(controller,robot);
    }


    AtomicInteger lastLight = new AtomicInteger(0);
     AtomicBoolean updateLight = new AtomicBoolean(true);

     void startLightControl(Hub controller, Hub robot) {
        Motor lightControl = (Motor)controller.getDevice(Port.A);
        UltrasonicSensor ultraLight = (UltrasonicSensor) robot.getDevice(Port.F);
        DataSubscriber.registerSimpleListener(lightControl, Motor.MotorDataType.AbsolutePosition,newValue -> {
            int light = (Integer) newValue;
            if(light < 5 && light > -5) {
                light = 0;
            } else {
                light = Math.min(100,light);
                light = Math.max(-100,light);
            }
            if (light != lastLight.get()) {
                lastLight.set(light);
                updateLight.set(true);
            }
        });
        new Thread(()->{
            while(true) {
                if (updateLight.get()) {
                    updateLight.set(false);
                    Logger.info("Light Control " + lastLight.get());
                    int light = lastLight.get()*2;
                    if (light == 0) ultraLight.getControl().lightUp(0,0,0,0);
                    if (light < 0) {
                        light += -1;
                        int l = light;
                        int r = 0;
                        if (light < 100) {
                            l = 100;
                            r = l -100;
                        }
                        ultraLight.getControl().lightUp(r,l,0,0).send();
                    } else {
                        int l = light;
                        int r = 0;
                        if (light < 100) {
                            l = 100;
                            r = l -100;
                        }
                        ultraLight.getControl().lightUp(0,0,l,r).send();
                    }
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException ignored) {}
                }
                try {
                    Thread.sleep(20);
                } catch (InterruptedException ignored) {}
            }
        }).start();
    }

     AtomicInteger lastGrab = new AtomicInteger(0);
     AtomicBoolean updateGrab = new AtomicBoolean(true);

     void startGrabControl(Hub controller, Hub robot) {

        Motor grabControl = (Motor)controller.getDevice(Port.C);
        ConnectedDevice grabD = robot.getDevice(Port.B);
        if (grabD != null) {
                Motor grabPower = (Motor) grabD;
            DataSubscriber.registerSimpleListener(grabControl, Motor.MotorDataType.AbsolutePosition,newValue -> {
                int grab = (Integer) newValue;

                grab *= 0.5;
                grab = Math.min(40,grab);
                grab = Math.max(-20,grab);

                if (grab != lastGrab.get()) {
                    lastGrab.set(grab);
                    updateGrab.set(true);
                }
            });
            new Thread(()->{
                while(true) {
                    if (updateGrab.get()) {
                        updateGrab.set(false);
                        Logger.info("Grab Control " + lastGrab.get());
                        grabPower.getControl().goToRelativePosition(lastGrab.get(),40,false,StopType.BRAKE,100,100).sendAsync();
                        Logger.info("Set Grab");
                        try {
                            Thread.sleep(100);
                        } catch (InterruptedException ignored) {}
                    }
                    try {
                        Thread.sleep(20);
                    } catch (InterruptedException ignored) {}
                }
            }).start();
        }

    }


     AtomicBoolean updateDistance = new AtomicBoolean(false);
     AtomicInteger lastDist = new AtomicInteger(0);

     void startDistanceReport(Hub controller,Hub robot) {
        DataSubscriber.registerSimpleListener((UltrasonicSensor)robot.getDevice(Port.F), UltrasonicSensor.UltrasonicSensorDataType.Distance, newValue -> {
            if ((Integer) newValue != lastDist.get()) {
                updateDistance.set(true);
                lastDist.set((Integer) newValue);
            }
        });

        new Thread(()-> {
            while (true) {
                if (updateDistance.get()) {
                    updateDistance.set(false);
                    int distance = ((UltrasonicSensor) robot.getDevice(Port.F)).getDistance();
                    if (distance > 200) {
                        robot.getControl().display().text("N").sendAsync();
                        controller.getControl().display().text("N").sendAsync();
                    } else {
                        Image image = fromNumber(distance);
                        robot.getControl().display().image(image).sendAsync();
                        controller.getControl().display().image(image).sendAsync();
                    }
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException ignored) {
                    }
                }
                try {
                    Thread.sleep(20);
                } catch (InterruptedException ignored) {
                }
            }
        }).start();
    }

     AtomicInteger lastPow = new AtomicInteger(0);
     AtomicBoolean updatePow = new AtomicBoolean(false);

     void startPowerControl(Hub controller,Hub robot) {
        Motor powerControl = (Motor)controller.getDevice(Port.B);
        Motor leftPower = (Motor)robot.getDevice(Port.C);
        Motor rightPower = (Motor)robot.getDevice(Port.E);
        DataSubscriber.registerSimpleListener(powerControl, Motor.MotorDataType.AbsolutePosition,newValue -> {
            int power = (Integer) newValue+45;
            if(power < 5 && power > -5) {
                power = 0;
            } else {
                power *= -2;
                power = Math.min(100,power);
                power = Math.max(-100,power);
            }
            if (power != lastPow.get()) {
                lastPow.set(power);
                updatePow.set(true);
            }
        });
        new Thread(()-> {
            while (true) {
                if (updatePow.get()) {
                    updatePow.set(false);
                    int lastPower = lastPow.get();
                    if (lastPower == 0) {
                        robot.getControl().move().stop(leftPower,rightPower,StopType.BRAKE).send();
                    } else {
                        robot.getControl().move().startPowers(leftPower,rightPower,lastPower,lastPower,100).send();
                    }
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException ignored) {
                    }
                }
                try {
                    Thread.sleep(20);
                } catch (InterruptedException ignored) {
                }
            }
        }).start();
    }

     AtomicInteger lastTurn = new AtomicInteger(0);
     AtomicBoolean updateTurn = new AtomicBoolean(true);

     void startTurnControl(Hub controller,Hub robot) {
        Motor turnControl = (Motor)controller.getDevice(Port.E);
        Motor turnPower = (Motor)robot.getDevice(Port.A);
        DataSubscriber.registerSimpleListener(turnControl, Motor.MotorDataType.AbsolutePosition,newValue -> {
            int turn = (Integer) newValue;
            if(turn < 5 && turn > -5) {
                turn = 0;
            } else {
                turn *= 2;
                turn = Math.min(120,turn);
                turn = Math.max(-120,turn);
            }
            if (turn != lastTurn.get()) {
                lastTurn.set(turn);
                updateTurn.set(true);
            }
        });
        new Thread(()->{
            while(true) {
                if (updateTurn.get()) {
                    updateTurn.set(false);
                    Logger.info("Turn Control " + lastTurn.get());
                    turnPower.getControl().goToRelativePosition(lastTurn.get(),40,false,StopType.BRAKE,100,100).send();
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException ignored) {}
                }
                try {
                    Thread.sleep(20);
                } catch (InterruptedException ignored) {}
            }
        }).start();
    }


     AtomicBoolean tryNormalizePow = new AtomicBoolean(false);
     AtomicBoolean tryNormalizeTurn = new AtomicBoolean(false);
     final int moveDetect = 1;

     void startNormalization(Hub controller) {
        Motor powerControl = (Motor)controller.getDevice(Port.B);
        Motor turnControl = (Motor)controller.getDevice(Port.E);
        normalize(tryNormalizePow,powerControl,-45);
        //normalize(tryNormalizeTurn,turnControl,0);
    }

     void normalize(AtomicBoolean activator, Motor motor,int defaultPos) {
        DataSubscriber.registerSimpleListener(motor, Motor.MotorDataType.AbsolutePosition,newValue -> {
            activator.set(true);
        });
        new Thread(() -> {
            while (true) {
                if(activator.get()) {
                    int pos = motor.getPosition();
                    CompletableFuture<String> result = motor.getControl().goToPositionWithDirection(defaultPos,15,PathDirection.SHORTEST,false,StopType.COAST,10,10).sendAsync();
                    while (true) {
                        try {
                            Thread.sleep(350);
                        } catch (InterruptedException ignored) {}
                        if (result.isDone()) {
                            activator.set(false);
                            break;
                        }
                        if (motor.getPosition() <= pos - moveDetect || motor.getPosition() >= pos + moveDetect) {
                            pos = motor.getPosition();
                            continue;
                        } else {
                            motor.getControl().stop(StopType.BRAKE,100).send();
                            break;
                        }
                    }
                }
                try {
                    Thread.sleep(200);
                } catch (Exception ignored) {}
            }
        }).start();
    }



    /**
     * Generiert ein Image-Objekt, das eine Zahl von 0 bis 200 anzeigt.
     * * @param number Die anzuzeigende Zahl (0-200).
     * @return Ein neues Image-Objekt mit der dargestellten Zahl.
     * @throws IllegalArgumentException wenn die Zahl außerhalb von 0-200 liegt.
     */
    public  Image fromNumber(int number) {
        if (number < 0 || number > 200) {
            throw new IllegalArgumentException("Zahl muss zwischen 0 und 200 liegen!");
        }

        Image img = new Image();

        final int[][][] FONT = {
                {{1,1}, {1,0}, {1,0}, {1,0}, {1,1}}, // 0
                {{0,1}, {0,1}, {0,1}, {0,1}, {0,1}}, // 1
                {{1,1}, {0,1}, {1,1}, {1,0}, {1,1}}, // 2
                {{1,1}, {0,1}, {1,1}, {0,1}, {1,1}}, // 3
                {{1,0}, {1,0}, {1,1}, {0,1}, {0,1}}, // 4
                {{1,1}, {1,0}, {1,1}, {0,1}, {1,1}}, // 5
                {{1,1}, {1,0}, {1,1}, {1,0}, {1,1}}, // 6
                {{1,1}, {0,1}, {0,1}, {0,1}, {0,1}}, // 7
                {{1,1}, {1,1}, {1,1}, {1,1}, {1,1}}, // 8
                {{1,1}, {1,1}, {1,1}, {0,1}, {1,1}}  // 9
        };

        if (number < 10) {
            int digit = number;
            drawDigit(img, FONT[digit], 1, 9);
        }
        else if (number < 100) {
            int tens = number / 10;
            int ones = number % 10;
            drawDigit(img, FONT[tens], 0, 9);
            drawDigit(img, FONT[ones], 3, 7);
        }
        else {
            int hundreds = number / 100;
            int tens = (number / 10) % 10;
            int ones = number % 10;
            for (int y = 0; y < 5; y++) {
                img.setPixel(0, y, 9);
                if (hundreds == 2) {
                    img.setPixel(1, y, 9);
                }
            }
            drawDigit(img, FONT[tens], hundreds == 1 ? 2 : 3, 6);
            drawDigit(img, FONT[ones], 4, 3);
        }

        return img;
    }


    private void drawDigit(Image img, int[][] digitPattern, int startX, int brightness) {
        for (int y = 0; y < 5; y++) {
            for (int x = 0; x < digitPattern[y].length; x++) {
                if (startX + x > 4) continue;
                if (digitPattern[y][x] == 1) {
                    img.setPixel(startX + x, y, brightness);
                }
            }
        }
    }
}

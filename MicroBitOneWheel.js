let MIN = 1000;
let MAX = 2000;
let balanced = 1500;

basic.forever(function () {

    //if start seqence = done, then
    let y = input.acceleration(Dimension.Y);
    y = Math.map(y, -1023, 1023, 1000, 2000);

    serial.writeLine("value");
    serial.writeNumber(y);

    //if board tutching ground, turn off
    //if y < MIN{
    // y=balanced;
    // }
    // if y > MAX {
    // y = balanced;
    // }

    //add PID controller
    pins.servoSetPulse(AnalogPin.P16, y)

    //hm, remove delay?
    control.waitMicros(100);
})

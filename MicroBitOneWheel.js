let MIN = -200;
let MAX = 200;
let balanced = 1500;

/*
basic.setMinMax(function(){
  //let run for 10 secs
  // tilt board from max left to max right
  let y = input.acceleration(Dimension.Y);
  if( y > MAX){
    MAX=y;
  }if(y<MIN){
    MIN=y;
  }

  control.waitMicros(100);
})
*/
basic.forever(function () {

    //if start seqence = done, then
    // values range from -1023 to 1023
    let y = input.acceleration(Dimension.Y);

    // serial.writeLine("value");
    // serial.writeNumber(y);

    //if board tutching ground, turn off
    // EXTEND RANGE BEFORE TURNING OFF
    if (y < MIN){
      if(y<MIN-50)
      {y=balanced;}
      else{y=1000;}
    }else if( y > MAX) {
      if(y>MAX+50)
      {y = balanced;}
      else{y=2000;}
    }else{
      y = Math.map(y, MAX, MIN, 1000, 2000);
    }

    //add PID controller UPDATE ( USE PID CONTROLLER IN VESC)
    pins.servoSetPulse(AnalogPin.P16, y)

    //hm, remove delay?
    // control.waitMicros(100);
})

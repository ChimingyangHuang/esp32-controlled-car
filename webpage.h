const char body[] PROGMEM = R"===( 
<!DOCTYPE html>  
<html> 
  <head>
    <style>
      body {
        display: flex;
        flex-direction: column;
        align-items: center;
        font-family: Arial, sans-serif;
      }
      h1 {
        text-align: center;
      }
      .control-container {
        display: flex;
        flex-direction: column;
        align-items: center;
        margin-bottom: 20px;
      }
      #speedSlider {
        writing-mode: bt-lr;
        -webkit-appearance: slider-vertical;
        width: 10px;
        height: 200px;
        margin: 10px;
      }
      .button-container {
          font-size: 16px; /* Increase the text size */
      }
      .controls-guide {
        margin-top: 20px;
        padding: 10px;
        background-color: #f0f0f0;
        border-radius: 5px;
      }
    </style>
  </head>
  <body>
    <h1>Motor Control</h1>
    
    <div class="control-container">
      <p>Speed (RPM):</p>
      <input type="range" id="speedSlider" min="20" max="120" onchange="updateSpeed()" value="50">
      <span id="speedDisplay">50</span> RPM
    </div>

    <div class="button-container">
      <button type="button" onclick="setForward()">Forward</button>
      <button type="button" onclick="setReverse()">Reverse</button>
      <button type="button" onclick="stopMotor()">Stop</button>
      <button type="button" onclick="turnLeft()">turnLeft</button>
      <button type="button" onclick="turnRight()">turnRight</button>
    </div>

    <div class="button-container">
      <button type="button" onclick="wallFollowingRight()">Start Wall Following Right Hand</button>
      <button type="button" onclick="endwallFollowingRight()">End Wall Following Right Hand</button>
      <button type="button" onclick="startServo()">Start Servo</button>
      <button type="button" onclick="endServo()">End Servo</button>
    </div>

    <div class="button-container">
      <button type="button" onclick="hitButton()">Start HitButton</button>
      <button type="button" onclick="endHitButton()">End HitButton</button>
    </div>

    <div class="controls-guide">
      <h3>Keyboard Controls:</h3>
      <p>W - Forward (Hold to move)</p>
      <p>S - Reverse (Hold to move)</p>
      <p>A/D - Turn Left/Right</p>
      <p>Q/E - Decrease/Increase Speed</p>
      <p>R - Emergency Stop</p>
    </div>
    
    <script>
      // Keep track of movement keys being pressed
      let keyStates = {
        w: false,
        s: false,
        a: false,
        d: false,
      };

      function wallFollowingRight(){
        var xhttp = new XMLHttpRequest();
        xhttp.open('GET', 'mode?value=' + 0, true); 
        xhttp.send();
      }

      function endwallFollowingRight(){
        var xhttp = new XMLHttpRequest();
        xhttp.open('GET', 'mode?value=' + 1, true); 
        xhttp.send();
      }

      function startServo(){
        var xhttp = new XMLHttpRequest();
        xhttp.open('GET', 'mode?value=' + 2, true); 
        xhttp.send();
      }

      function endServo(){
        var xhttp = new XMLHttpRequest();
        xhttp.open('GET', 'mode?value=' + 3, true); 
        xhttp.send();
      }

      function startHitLeft(){
        var xhttp = new XMLHttpRequest();
        xhttp.open('GET', 'mode?value=' + 4, true); 
        xhttp.send();
      }
      
      function endHitLeft(){
        var xhttp = new XMLHttpRequest();
        xhttp.open('GET', 'mode?value=' + 5, true); 
        xhttp.send();
      }

      function rotate0(){
        var xhttp = new XMLHttpRequest();
        xhttp.open('GET', 'mode?value=' + 6, true); 
        xhttp.send();
      }
      function rotate90(){
        var xhttp = new XMLHttpRequest();
        xhttp.open('GET', 'mode?value=' + 7, true); 
        xhttp.send();
      }
      function rotate180(){
        var xhttp = new XMLHttpRequest();
        xhttp.open('GET', 'mode?value=' + 8, true); 
        xhttp.send();
      }
      function rotate270(){
        var xhttp = new XMLHttpRequest();
        xhttp.open('GET', 'mode?value=' + 9, true); 
        xhttp.send();
      }
      
      function hitButton(){
        var xhttp = new XMLHttpRequest();
        xhttp.open('GET', 'mode?value=' + 10, true); 
        xhttp.send();
      }

      function endHitButton(){
        var xhttp = new XMLHttpRequest();
        xhttp.open('GET', 'mode?value=' + 11, true); 
        xhttp.send();
      }
      
      function updateSpeed() {
        var xhttp = new XMLHttpRequest();
        var speed = document.getElementById('speedSlider').value;
        document.getElementById('speedDisplay').innerHTML = speed;
        xhttp.open('GET', 'speed?value=' + speed, true); 
        xhttp.send();
      }
      
      function setForward() {
        var xhttp = new XMLHttpRequest();
        xhttp.open('GET', 'direction?dir=1', true);  
        xhttp.send();
      }

      function setReverse() {
        var xhttp = new XMLHttpRequest();
        xhttp.open('GET', 'direction?dir=-1', true);  
        xhttp.send();
      }

      function stopMotor() {
        var xhttp = new XMLHttpRequest();
        xhttp.open('GET', 'direction?dir=0', true);  
        xhttp.send();
      }

      function turnLeft() {
        var xhttp = new XMLHttpRequest();
        xhttp.open('GET', 'turn?dir=1', true);  
        xhttp.send();
      }

      function turnRight() {
        var xhttp = new XMLHttpRequest();
        xhttp.open('GET', 'turn?dir=2', true);  
        xhttp.send();
      }

      

      // Function to adjust speed slider
      function adjustSpeed(increment) {
        const slider = document.getElementById('speedSlider');
        const currentSpeed = parseInt(slider.value);
        const step = 5; // Speed adjustment step
        const newSpeed = Math.max(30, Math.min(120, currentSpeed + (increment ? step : -step)));
        slider.value = newSpeed;
        updateSpeed();
      }

      // Check if we should be moving
      function updateMovement() {
        if (keyStates.w) {
          setForward();

        } else if (keyStates.s) {
          setReverse();

        } else if(keyStates.a){
          turnLeft();

        }else if(keyStates.d){
          turnRight();

        }
      }

      // Keyboard down event listener
      document.addEventListener('keydown', function(event) {
        switch(event.key.toLowerCase()) {
          case 'w':
            if (!keyStates.w) {
              keyStates.w = true;
              keyStates.s = false;
              keyStates.a = false;
              keyStates.d = false;
              updateMovement();
            }
            break;
          case 's':
            if (!keyStates.s) {
              keyStates.w = false;
              keyStates.s = true;
              keyStates.a = false;
              keyStates.d = false;
              updateMovement();
            }
            break;
          case 'a':
            if (!keyStates.a) {
              keyStates.w = false;
              keyStates.s = false;
              keyStates.a = true;
              keyStates.d = false;
              updateMovement();
            }
            break;
          case 'd':
            if (!keyStates.d) {
              keyStates.w = false;
              keyStates.s = false;
              keyStates.a = false;
              keyStates.d = true;
              updateMovement();
            }
            break;
          case 'q':
            adjustSpeed(false); // Decrease speed
            break;
          case 'e':
            adjustSpeed(true);  // Increase speed
            break;
          case 'c':
            startServo();
            break;
          case 'v':
            endServo();
            break;
          case 'r':
            stopMotor();
            keyStates.w = false;
            keyStates.s = false;
            break;
        }
      });

      // Keyboard up event listener
      document.addEventListener('keyup', function(event) {
        switch(event.key.toLowerCase()) {
          case 'w':
            keyStates.w = false;
            stopMotor();

            break;
          case 's':
            keyStates.s = false;
            stopMotor();

            break;
          case 'a':
            keyStates.a = false;
            stopMotor();

            break;
          case 'd':
            keyStates.d = false;
            stopMotor();

            break;
        }
      });
    </script>
  </body>
</html>
)===";
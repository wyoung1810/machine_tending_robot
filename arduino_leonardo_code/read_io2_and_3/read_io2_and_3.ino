void setup() {
  Serial.begin(9600);
  pinMode(5, INPUT);  // Set pin 12 as input
  pinMode(6, INPUT);  // Set pin 13 as input
}

void loop() {
  int loadpin = digitalRead(5);
  int unloadpin = digitalRead(6);
  
  // Send both pin states as a comma-separated string
  Serial.print(loadpin);
  Serial.print(",");
  Serial.println(unloadpin);
  
  delay(100);  // Read pins 10 times per second
}


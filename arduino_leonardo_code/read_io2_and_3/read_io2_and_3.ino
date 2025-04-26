void setup() {
  Serial.begin(9600);
  pinMode(10, INPUT);  // Set pin 12 as input
  pinMode(11, INPUT);  // Set pin 13 as input
}

void loop() {
  int pin10Value = digitalRead(10);
  int pin11Value = digitalRead(11);
  
  // Send both pin states as a comma-separated string
  Serial.print(pin10Value);
  Serial.print(",");
  Serial.println(pin11Value);
  
  delay(100);  // Read pins 10 times per second
}


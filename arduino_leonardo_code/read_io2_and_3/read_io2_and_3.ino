void setup() {
  Serial.begin(9600);
  pinMode(2, INPUT);  // Set pin 2 as input
  pinMode(3, INPUT);  // Set pin 3 as input
}

void loop() {
  digitalWrite(2, HIGH);
  int pin2Value = digitalRead(2);
  int pin3Value = digitalRead(3);
  
  // Send both pin states as a comma-separated string
  Serial.print(pin2Value);
  Serial.print(",");
  Serial.println(pin3Value);
  
  delay(100);  // Read pins 10 times per second
}


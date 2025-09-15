unsigned int count = 0;
bool done = false;

void setup() {
  pinMode(7, OUTPUT);
  digitalWrite(7, 0);
  delay(1000);
  digitalWrite(7, 1);
}

void loop() {
  if (!done) {
    while (count < 6) {   
      digitalWrite(7, 0);
      delay(100);
      digitalWrite(7, 1);
      delay(100);
      count++;
    }
    done = true;
  }
}

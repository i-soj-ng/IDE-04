void setup() {
  // put your setup code here, to run once:
  pinMode(7, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
    digitalWrite(7, 0);
    delay(1000);
    int i;
    i=0;
    while(i<5){
      digitalWrite(7, 1);
      delay(200);
      digitalWrite(7, 0);
      delay(200);
      i = i+1;
    }

    while(1){
      digitalWrite(7, 1);
    }
}

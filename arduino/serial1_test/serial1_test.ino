void setup() {
  // put your setup code here, to run once:
Serial1.begin(9600);
Serial1.println('start');
}

void loop() {
  Serial1.print('.');
  // put your main code here, to run repeatedly:
delay(100);
}

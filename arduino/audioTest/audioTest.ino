#define SPEAKER_PIN 3

void setup() {
  analogWriteFrequency(SPEAKER_PIN, 20);
  // put your setup code here, to run once:
analogWrite(SPEAKER_PIN, 1);
}

void loop() {

  analogWrite(SPEAKER_PIN, 1);
  delay(100);
  analogWrite(SPEAKER_PIN, 0);
  delay(100);
  // put your main code here, to run repeatedly:
}

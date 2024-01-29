void setup() {
  Serial.begin(115200); 
}
void loop() {
  const byte pinNumber = 34;
  int inputValue = analogRead(pinNumber);
  float inputVoltage = 3.3 / 4096 * inputValue;
  float adjustedInputValue = analogReadAdjusted(pinNumber);
  float adjustedInputVoltage = 3.3 / 4096 * adjustedInputValue;

  Serial.print("Input: ");
  Serial.print(inputValue);
  Serial.print(" | I new: ");
  Serial.print(adjustedInputValue, 3);
  Serial.print(" | Voltage: ");
  Serial.print(inputVoltage, 3);
  Serial.print(" | V new: ");
  Serial.print(adjustedInputVoltage, 3);
  Serial.print(" | Delta %: ");
  Serial.println((adjustedInputValue - inputValue) / inputValue * 100, 2);
  delay(500);
}
// Function for the linear adjustment of the ADC.
double analogReadAdjusted(byte pinNumber){
  const double f1 = 6.2368347718472492e+001;
  const double f2 =  2.2747034934713861e+000;
  const double f3 = -7.9994558213056788e-003;
  const double f4 =  2.4014349910239734e-005;
  const double f5 = -4.0871069658643206e-008;
  const double f6 =  4.2902463940351207e-011;
  const double f7 = -2.9061500055422066e-014;
  const double f8 =  1.2931238462787567e-017;
  const double f9 = -3.7531431973193992e-021;
  const double f10 =  6.8363850661158004e-025;
  const double f11 = -7.0913447289332223e-029;
  const double f12 =  3.1946783356275369e-033;
  const int loops = 40;
  const int loopDelay = 50;
  int counter = 1;
  int inputValue = 0;
  double totalInputValue = 0;
  double averageInputValue = 0;
  // Loop to get the average of different analog values.
  for (counter = 1; counter <= loops; counter++) {
    inputValue = analogRead(pinNumber);
    totalInputValue += inputValue;
    delay(loopDelay);
  }
  averageInputValue = totalInputValue / loops;
  return f1+f2*pow(averageInputValue,1)+f3*pow(averageInputValue,2)+
  f4*pow(averageInputValue,3)+f5*pow(averageInputValue,4)+f6*pow(averageInputValue,5) 
  +f7*pow(averageInputValue,6)+f8*pow(averageInputValue,7)+f9*pow(averageInputValue, 8)
  +f10* pow(averageInputValue,9)+f11*pow(averageInputValue,10)+f12*pow(averageInputValue,11);
}
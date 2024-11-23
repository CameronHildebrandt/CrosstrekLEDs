int ap0 = A0;
int val = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  val = analogRead(ap0);

//  (val+125)/175) // 0-6 (0 brightest)
  Serial.println(((val+125)/175)*(-90/6)+100); // 100-10 (100 brightest)

//  Serial.println( ((-3*val)/35) + (625/7) ); // Inaccurate, but maybe faster compute?
  
//  if(val > 925) {
//    Serial.println("[*      ]");
//  } else if(val > 750) {
//    Serial.println("[**     ]");
//  } else if(val > 575) {
//    Serial.println("[***    ]");
//  } else if(val > 400) {
//    Serial.println("[****   ]");
//  } else if(val > 225) {
//    Serial.println("[*****  ]");
//  } else if(val > 50) {
//    Serial.println("[****** ]");
//  } else {
//    Serial.println("[**MAX**]");
//  }
  
}

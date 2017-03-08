void pixyUpdate() {
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  
  blocks = pixy.getBlocks();
  
  if (blocks) {
    i++;
   
    sprintf(buf, "Detected %d:\n", blocks);
    Serial.print(buf);
    for (j=0; j<blocks; j++) {

      if(pixy.blocks[j].width > 15 && pixy.blocks[j].height > 15) {
        sprintf(buf, "  block %d: ", j);
        Serial.print(buf); 
        pixy.blocks[j].print();

        if(pixy.blocks[j].signature == 2) {
          if(pixy.blocks[j].width <= 120) {
            if(pixy.blocks[j].x < 160) {
              // move right
              leftBork();
              motor_setDir(0, MOTOR_DIR_FWD);
              motor_setSpeed(0, 255);
              motor_setDir(1, MOTOR_DIR_FWD);
              motor_setSpeed(1, 60);
            } else {
              // move left
              leftBork();
              motor_setDir(0, MOTOR_DIR_FWD);
              motor_setSpeed(0, 60);
              motor_setDir(1, MOTOR_DIR_FWD);
              motor_setSpeed(1, 255);
            }
          }
        }
        
      }
      
    }
    
  }
}



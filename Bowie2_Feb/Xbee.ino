
void xbeeSend() {
  
  sprintf(message_tx,"Weewoo %d", 42);
  ZBTxRequest zbtx = ZBTxRequest(addr_controller, (uint8_t *)message_tx, strlen(message_tx));
  xbee.send(zbtx);

  if (xbee.readPacket(250)) {
    if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
      xbee.getResponse().getZBTxStatusResponse(txStatus);
      if (txStatus.getDeliveryStatus() == SUCCESS) {
        msg_tx_count++;
        Serial.print(msg_tx_count);
        Serial.println(" msg sent");
      } else {
        msg_tx_err++;
        Serial.print(msg_tx_err);
        Serial.println(" msg did not send, encountered an error");
      }
    }
  }
  
}

bool xbeeRead() {
  
  xbee.readPacket();

  if (xbee.getResponse().isAvailable()) { // we got something
    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) { // got an rx packet
      xbee.getResponse().getZBRxResponse(rx);

      XBeeAddress64 senderLongAddress = rx.getRemoteAddress64();
      
      if(DEBUG) {
        Serial.print(msg_rx_count);
        Serial.print(" Received data from ");
        print32Bits(senderLongAddress.getMsb());
        print32Bits(senderLongAddress.getLsb());
        Serial.println(": ");
      }  
      
      for(int i=0; i<64; i++) {
        message_rx[i] = ' ';
      }

      // ascii representation of text sent through xbee
      for (int i=0; i <rx.getDataLength(); i++){
        if (!iscntrl(rx.getData()[i])) {
          message_rx[i] = (char)rx.getData()[i];
          if(DEBUG) Serial.write(message_rx[i]);
        }
      }
      if(DEBUG) Serial.println();
      
      msg_rx_count++;
      return true;
    }
  }

  return false;
  
}


// these routines are just to print the data with
// leading zeros and allow formatting such that it
// will be easy to read.
void print32Bits(uint32_t dw){
  print16Bits(dw >> 16);
  print16Bits(dw & 0xFFFF);
}

void print16Bits(uint16_t w){
  print8Bits(w >> 8);
  print8Bits(w & 0x00FF);
}

void print8Bits(byte c){
  uint8_t nibble = (c >> 4);
  if (nibble <= 9)
    Serial.write(nibble + 0x30);
  else
    Serial.write(nibble + 0x37);
      
  nibble = (uint8_t) (c & 0x0F);
  if (nibble <= 9)
    Serial.write(nibble + 0x30);
  else
    Serial.write(nibble + 0x37);
}



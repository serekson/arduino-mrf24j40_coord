#include <Arduino.h>
#include <mrf24j40_coord.h>

MRFCoord::MRFCoord(int pin_cs, int pin_int)
  : MRFDriver(pin_cs, pin_int)
{
  //nothing here for now
}

void MRFCoord::init(void){
  write_short(MRF_GPIO, 0x00);
  write_short(MRF_TRISGPIO, 0x00);
  write_long(MRF_TESTMODE, 0x0F);
  
  write_short(MRF_PACON0, 0x29);
  write_short(MRF_PACON1, 0x02);
  write_short(MRF_PACON2, 0x98); // Initialize FIFOEN = 1 and TXONTS = 0x6.
  write_short(MRF_TXSTBL, 0x95); // Initialize RFSTBL = 0x9.

  write_long(MRF_RFCON0, 0x03); // Initialize RFOPT = 0x03.
  write_long(MRF_RFCON1, 0x02); // Initialize VCOOPT = 0x02.
  write_long(MRF_RFCON2, 0x80); // Enable PLL (PLLEN = 1).
  write_long(MRF_RFCON6, 0x90); // Initialize TXFIL = 1 and 20MRECVR = 1.
  write_long(MRF_RFCON7, 0x80); // Initialize SLPCLKSEL = 0x2 (100 kHz Internal oscillator).
  write_long(MRF_RFCON8, 0x10); // Initialize RFVCO = 1.
  write_long(MRF_SLPCON1, 0x21); // Initialize CLKOUTEN = 1 and SLPCLKDIV = 0x01.

  // Configuration for nonbeacon-enabled devices (see Section 3.8 Beacon-Enabled and
  // Nonbeacon-Enabled Networks)
  write_short(MRF_TXMCR, 0x1C);  //Clear Slotted mode
  EnergyDetect();
  write_short(MRF_RXMCR, 0x08); //Set as PC
  //write_short(MRF_RXMCR, 0x09); //PC, PROMI
  write_short(MRF_ORDER, 0xFF); //BO=15 and SO=15

  //Security
  set_AES_key(); //install the tx and rx security key
  //write_short(MRF_SECCON0,0x12); //enable AES-CCM-128 on the TXFIFO and RXFIFO
  //write_short(MRF_SECCON0,0x09); //AES-CTR
  write_short(MRF_SECCON0,0); //NONE

  write_short(MRF_BBREG2, 0x80); // Set CCA mode to ED
  write_short(MRF_CCAEDTH, 0x60); // Set CCA ED threshold.
  write_short(MRF_BBREG6, 0x40); // Set appended RSSI value to RXFIFO.

  // Initialize interrupts
  write_long(MRF_SLPCON0, 0x01); //Interrupt on falling edge and disable sleep clock
  write_short(MRF_INTCON, 0xE6); //Enable SEC, RX, and TX Interrupts

  set_channel(7);
  write_long(MRF_RFCON3, 0x40); //Select TX Power
  write_short(MRF_RFCTL, 0x04); //Reset RF state machine.
  write_short(MRF_RFCTL, 0x00);

  delay(1); //delay at least 192usec

  writeAddress();

  local.pan[0] = 0xA1;
  local.pan[1] = 0xCB;
  writePAN(local.pan);
  local.addr16[0] = 0xFE;
  local.addr16[1] = 0xFF;
  write_addr16(local.addr16);
  client_cnt = 0;
  local.time = millis();

  _rx_count = 0;
}

void MRFCoord::coord_loop(void) {
  uint32_t current_time;
  
  if(int_mrf) {
    proc_interrupt();
  }
  
  if(_rx_count>0) {
    rx_packet();
    _rx_count--;
  }
  
  current_time = millis();

  if(current_time - local.time > 30000) {
    tx_beacon();
  }
  
  /*
  for(i=0;i<client_cnt;i++) {
    //Serial.print("time: ");
	//Serial.print(client_list[i].time);
    switch(client_list[i].status) {
      case 0:
	    break;
	  case 4:
        if((current_time - client_list[i].time) > 30000) {
          tx_data_cmd(client_list[i].addr, 0x06);  //heartbeat request
		  client_list[i].status = 5;
        }
		break;
      case 5:
	    if((current_time - client_list[i].time) > 40000) {
		  client_list[i].status = 0;
		  Serial.print("Lost Client: ");
		  printAddress(client_list[i].addr);
		}
		break;
      default:
	    Serial.print("client: ");
		Serial.println(i);
		Serial.print("status: ");
		Serial.println(client_list[i].status);
    }
  }
  */
}

void MRFCoord::udp_to_mrf(int packetSize, byte* packetBuffer) {
  DeviceAddress dest_addr;
  uint8_t mrf_cmd;
  uint8_t rims_cmd;
  int i;
  int j;

  for(j=0;j<8;j++) {
    dest_addr[j] = packetBuffer[8-j];
  }
  mrf_cmd = packetBuffer[9];
  rims_cmd = packetBuffer[10];

  //printAddress(dest_addr);
  //Serial.print("mrf_cmd: ");
  //Serial.println(mrf_cmd, DEC);
  //Serial.print("rims_cmd: ");
  //Serial.println(rims_cmd, DEC);

  i=0;
  write_long(i++,21); //header length
  write_long(i++,21+packetSize-9); //frame length

  // 0 | PANID comp | ack| frame pending | security | frame type <3>
  //write_long(i++,0b01101001);
  write_long(i++,0b01100001);

  // src mode <2> | frame ver <2> | dest mode <2> | 00
  write_long(i++,0b11001100);

  write_long(i++,_seq_num); //sequence number

  //write panid
  for(j=0;j<2;j++) {
    write_long(i++,local.pan[j]);
  }

  //write dest addr
  for(j=0;j<8;j++){
	  write_long(i++,dest_addr[j]);
  }

  //write src addr
  for(j=0;j<8;j++) {
    write_long(i++,local.addr[j]);
  }
  //end of header (21 bytes)

  write_long(i++,mrf_cmd);
  write_long(i++,rims_cmd);

  for(j=11;j<packetSize;j++) {
    write_long(i++,packetBuffer[j]); //load packet data
    //Serial.println(packetBuffer[j], HEX);
  }

  //Serial.print("mrf cnt: ");
  //Serial.println(i);

  tx_ready();
}

void MRFCoord::send_clients(void) {
  uint8_t ptr;
  uint8_t i;
  uint8_t j;
  
  union {
    byte asBytes[4];
    uint32_t asUINT32;
  } byte2var;
  
  ptr=0;
  udp_buffer[ptr++] = 0x01; //udp cmd type (mrf cmd)
  udp_buffer[ptr++] = 0x01; //udp cmd type (mrf cmd)
  
  udp_buffer[ptr++] = client_cnt;
  
  for(i=0;i<client_cnt;i++) {
    for(j=0;j<8;j++) {
      udp_buffer[ptr++] = client_list[i].addr[7-j];
    }
	
	for(j=0;j<2;j++) {
      udp_buffer[ptr++] = client_list[i].pan[1-j];
    }
	
	byte2var.asUINT32 = client_list[i].time;
	for(j=0;j<4;j++) {
      udp_buffer[ptr++] = byte2var.asBytes[j];
	}
	
	udp_buffer[ptr++] = client_list[i].status;
	udp_buffer[ptr++] = client_list[i].rssi;
	udp_buffer[ptr++] = client_list[i].lqi;
  }
  
  _udp_pending = ptr;
}

void MRFCoord::tx_beacon(void) {
  int i;
  int j;

  i=0;
  write_long(i++,13); //header length
  write_long(i++,16); //frame length

  // 0 | PANID comp | ack| frame pending | security | frame type <3>
  //write_long(i++,0b00001000);
  write_long(i++,0b00000000);

  // src mode <2> | frame ver <2> | dest mode <2> | 00
  write_long(i++,0b11000000);

  write_long(i++,_seq_num); //sequence number

  //write panid
  for(j=0;j<2;j++) {
    write_long(i++,local.pan[j]);
  }

  //write src addr
  for(j=0;j<8;j++) {
    write_long(i++,local.addr[j]);
  }

  // superframe order <4> | beacon order <4>
  write_long(i++,0b00000000); //this field is ignored
  // assoc permit | PC | 0 | bat life ext | final CAP <4>
  write_long(i++,0b11000000); //most of this is ignored
  write_long(i++,0x00); //Hops to PC

  tx_ready();

  //Serial.println("TX BEACON");
  local.time = millis();
}

void MRFCoord::tx_assoc_resp(DeviceAddress dest_addr) {
  int i;
  int j;

  i=0;
  write_long(i++,23); //header length
  write_long(i++,27); //frame length

  // 0 | PANID comp | ack| frame pending | security | frame type <3>
  //write_long(i++,0b00101011);
  write_long(i++,0b00100011);

  // src mode <2> | frame ver <2> | dest mode <2> | 00
  write_long(i++,0b11001100);

  write_long(i++,_seq_num);  // sequence number 1
  write_long(i++,0xFF); //dest PANID low (broadcast)
  write_long(i++,0xFF); //dest PANID high (broadcast)

  for(j=0;j<8;j++) {
    write_long(i++,dest_addr[j]);
  }

  //write panid
  for(j=0;j<2;j++) {
    write_long(i++,local.pan[j]);
  }

  //write src addr
  for(j=0;j<8;j++) {
    write_long(i++,local.addr[j]);
  }

  write_long(i++,0x02); //MAC Command Frame (association request)
  write_long(i++,0xFE); //short addr low
  write_long(i++,0xFF); //short addr high
  write_long(i++,0x00); //assoc status (0x00 is success)

  tx_ready();

  Serial.print("Join: ");
  printAddress(dest_addr);
  
  //add to client list
  j=0;
  for(i=0;i<client_cnt;i++) {
    if(compareAddress(client_list[i].addr, dest_addr)) {
	  Serial.print("Old Client: ");
	  client_list[i].time = millis();
	  client_list[i].status = 4;
	  j=1;
	}
  }
  
  if(j==0) {
    Serial.print("New Client: ");
    setAddress(client_list[client_cnt].addr, dest_addr);
    setPAN(client_list[client_cnt].pan, local.pan);
    client_list[client_cnt].time=millis();
    client_list[client_cnt].status=4;
    client_cnt++;
  }

  printAddress(dest_addr);
  
  //Serial.println("TX ASSOC RESP");
}

void MRFCoord::tx_data_cmd(DeviceAddress dest_addr, byte data_cmd) {
  int i;
  int j;

  i=0;
  write_long(i++,21); //header length
  write_long(i++,22); //frame length

  // 0 | PANID comp | ack| frame pending | security | frame type <3>
  //write_long(i++,0b01101001);
  write_long(i++,0b01100001);
  // src mode <2> | frame ver <2> | dest mode <2> | 00
  write_long(i++,0b11001100);

  write_long(i++,_seq_num); //sequence number

  //write panid
  for(j=0;j<2;j++) {
    write_long(i++,local.pan[j]);
  }

  //write dest addr
  for(j=0;j<8;j++){
	  write_long(i++,dest_addr[j]);
  }

  //write src addr
  for(j=0;j<8;j++) {
    write_long(i++,local.addr[j]);
  }
  //end of header (21 bytes)

  write_long(i++,data_cmd); //cmd id

  tx_ready();

  //Serial.print("TX DATA CMD: ");
  //Serial.println(data_cmd, HEX);
}

void MRFCoord::rx_packet(void) {
  uint8_t i;
  
  switch(packet.frm_ctrl1 & 0x07) {
    case 0x00:
      //rx_beacon();
	  //Serial.println("RX Beacon");
      break;
    case 0x01:
	  //Serial.println("RX Data");
      rx_data();
      break;
    case 0x02:
      //Serial.println("ack frame");
	  break;
    case 0x03:
	  //Serial.println("RX Mac");
      rx_mac();
      break;
    default:
      Serial.print("invalid frame type: ");
      Serial.println(packet.frm_ctrl1 & 0x07, HEX);
  }
  
  //update client list
  for(i=0;i<client_cnt;i++) {
    if(compareAddress(packet.src_addr,client_list[i].addr)) {
      client_list[i].time = millis();
	  client_list[i].status = 4;
	  client_list[i].rssi = packet.rssi;
	  client_list[i].lqi = packet.lqi;
    }
  }
}

void MRFCoord::rx_mac(void) {
  switch(packet.data[0]) {
    case 0x01:  //assoc req
      //Serial.println("RX ASSOC REQ");
      tx_assoc_resp(packet.src_addr);
      break;
    case 0x02:  //assoc resp
      Serial.println("ERR: RX ASSOC RESP");
      break;
    //case 0x03:  //dissoc not
    //case 0x04:  //data req
    //case 0x05:  //PAN ID Conflict
    //case 0x06:  //orphan not
    case 0x07:  //beacon req
      Serial.println("RX BEACON REQ");
      tx_beacon();
      break;
    //case 0x08:  //coord realign
    //case 0x09:  //GTS req
    default:
      Serial.print("MAC COMMAND ");
      Serial.print(packet.data[0], HEX);
      Serial.println(" NOT PROGRAMMED");
  }
}

void MRFCoord::rx_data(void) {
  uint8_t ptr;
  uint8_t i;

  switch(packet.data[0]) {
    case 0x05:  //rx heartbeat
      //Serial.print("heartbeat: ");
      //printAddress(packet.src_addr);
      break;
   case 0x09:  //mrf to udp
      ptr=0;
	  udp_buffer[ptr++] = 0x02; //udp cmd type (rims cmd)

	  for(i=0;i<8;i++) {
        udp_buffer[ptr++] = packet.src_addr[7-i];
	  }

	  for(i=1;i<packet.data_len;i++) {
	    udp_buffer[ptr++] = packet.data[i];
      }

	  _udp_pending = ptr;
	  break;
    default:
      Serial.print("Data Command ");
      Serial.print(packet.data[0], HEX);
      Serial.println(" is not valid");
  }
}
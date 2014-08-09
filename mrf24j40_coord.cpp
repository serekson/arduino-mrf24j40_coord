#include <Arduino.h>
#include <mrf24j40_coord.h>

MRFCoord::MRFCoord(int pin_cs, int pin_int)
  : MRFDriver(pin_cs, pin_int)
{
  //nothing here for now
}

void MRFCoord::init_coord(void) {
  init();
  
  //EnergyDetect();
  write_short(MRF_RXMCR, 0x08); //Set as PC
  //write_short(MRF_RXMCR, 0x09); //PC, PROMI
  write_short(MRF_ORDER, 0xFF); //BO=15 and SO=15

  write_pan(0xCBA1);
  write_addr16(0xFFFE);
  _beacon_timer = millis();
}

void MRFCoord::coord_loop(void) {
  uint32_t current_time = millis();

  if(current_time - _beacon_timer > 30000) {
    tx_beacon();
  }
  
  if(int_mrf) {
    proc_interrupt();
  }
  
  if(_rx_count>0) {
    rx_packet();
    _rx_count--;
  }
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
    write_long(i++,mrf_panid[j]);
  }

  //write dest addr
  for(j=0;j<8;j++){
	  write_long(i++,dest_addr[j]);
  }

  //write src addr
  for(j=0;j<8;j++) {
    write_long(i++,mrf_mac[j]);
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
    write_long(i++,mrf_panid[j]);
  }

  //write src addr
  for(j=0;j<8;j++) {
    write_long(i++,mrf_mac[j]);
  }

  // superframe order <4> | beacon order <4>
  write_long(i++,0b00000000); //this field is ignored
  // assoc permit | PC | 0 | bat life ext | final CAP <4>
  write_long(i++,0b11000000); //most of this is ignored
  write_long(i++,0x00); //Hops to PC

  tx_ready();

  //Serial.println("TX BEACON");
  _beacon_timer = millis();
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
    write_long(i++,mrf_panid[j]);
  }

  //write src addr
  for(j=0;j<8;j++) {
    write_long(i++,mrf_mac[j]);
  }

  write_long(i++,0x02); //MAC Command Frame (association request)
  write_long(i++,0xFE); //short addr low
  write_long(i++,0xFF); //short addr high
  write_long(i++,0x00); //assoc status (0x00 is success)

  tx_ready();

  Serial.print("Join: ");
  printAddress(dest_addr);
  //memcpy(client_list[client_cnt], dest_addr, 8);
  //setAddress(client_list[client_cnt].addr, dest_addr);
  client_list[client_cnt].time=millis();
  client_cnt++;

  //Serial.println("TX ASSOC RESP");
}

void MRFCoord::rx_packet(void) {	
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
      Serial.println(packet.frm_ctrl1, HEX);
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
      //Serial.println("RX BEACON REQ/TX BEACON");
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
  uint8_t i;
  uint8_t j;

  switch(packet.data[0]) {
    case 0x06:  //heartbeat request
      Serial.print("heartbeat REQ: ");
      printAddress(packet.src_addr);
      //tx_data_cmd(packet.src_addr, 0x05);
      break;
   case 0x09:  //mrf to udp
      i=0;
	  udp_buffer[i++] = 0x02; //udp cmd type (rims cmd)

	  for(j=0;j<8;j++) {
        udp_buffer[i++] = packet.src_addr[7-j];
	  }

	  for(j=1;j<packet.data_len;j++) {
	    udp_buffer[i++] = packet.data[j];
      }

	  _udp_pending = i;
	  break;
    default:
      Serial.print("Data Command ");
      Serial.print(packet.data[0], HEX);
      Serial.println(" is not valid");
  }
}
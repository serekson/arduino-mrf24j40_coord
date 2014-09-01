#ifndef _MRF24J40_COORD_H_INCLUDED
#define _MRF24J40_COORD_H_INCLUDED

#include <Arduino.h>
#include <mrf24j40_driver.h>

#define MRF_MAX_CLIENTS 0x04

class MRFCoord : public MRFDriver {
  private:
  byte client_cnt;
  Node client_list[MRF_MAX_CLIENTS];
  
  public:
  MRFCoord(int pin_cs, int pin_int);
  
  uint8_t _udp_pending;
  byte udp_buffer[64];
  
  void init(void);
  void coord_loop(void);
  void udp_to_mrf(int packetSize, byte* packetBuffer) ;
  void send_clients(void);
	
  void tx_beacon(void);
  void tx_assoc_resp(DeviceAddress dest_addr);
  void tx_data_cmd(DeviceAddress dest_addr, byte data_cmd);
  
  void rx_packet(void);
  void rx_mac(void);
  void rx_data(void);
};
#endif
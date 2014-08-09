#ifndef _MRF24J40_COORD_H_INCLUDED
#define _MRF24J40_COORD_H_INCLUDED

#include <Arduino.h>
#include <mrf24j40_driver.h>

#define MRF_MAX_CLIENTS 0x04

typedef struct {
  DeviceAddress addr;
  uint32_t time;
} Client2;

class MRFCoord : public MRFDriver {
  private:
  uint32_t _beacon_timer;
  
  uint8_t client_cnt;
  Client2 client_list[MRF_MAX_CLIENTS];
  
  public:
  MRFCoord(int pin_cs, int pin_int);
  
  uint8_t _udp_pending;
  byte udp_buffer[64];
  
  //uint8_t _rx_count;
  
  void init_coord(void);
  void coord_loop(void);
  void udp_to_mrf(int packetSize, byte* packetBuffer) ;
	
  void tx_beacon(void);
  void tx_assoc_resp(DeviceAddress dest_addr);
  
  void rx_packet(void);
  void rx_mac(void);
  void rx_data(void);
};
#endif